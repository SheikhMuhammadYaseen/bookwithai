from qdrant_client import QdrantClient
import google.generativeai as genai
from tenacity import retry, stop_after_attempt, wait_exponential
import logging
from . import config, agents

# Configure logging
logging.basicConfig(level=logging.INFO)

# Initialize clients
try:
    qdrant_client = QdrantClient(
        url=config.QDRANT_HOST,
        api_key=config.QDRANT_API_KEY,
    )
    logging.info("Qdrant client initialized successfully.")
except Exception as e:
    logging.error(f"Failed to initialize Qdrant client: {e}")
    # Handle the error appropriately, maybe exit or use a mock client
    qdrant_client = None

try:
    genai.configure(api_key=config.GEMINI_API_KEY)
    gemini_embedding_model = config.GEMINI_EMBED_MODEL
    gemini_chat_model = genai.GenerativeModel(config.GEMINI_CHAT_MODEL)
    logging.info("Gemini client configured successfully.")
except Exception as e:
    logging.error(f"Failed to configure Gemini client: {e}")
    gemini_chat_model = None

@retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
def embed_query(query: str):
    """Embeds a query using Gemini, with retry for transient errors."""
    try:
        result = genai.embed_content(
            model=gemini_embedding_model,
            content=query,
            task_type="retrieval_query"
        )
        return result['embedding']
    except Exception as e:
        logging.error(f"An unexpected error occurred during embedding: {e}")
        # Re-raise the exception to allow tenacity to handle retries
        raise

def search_qdrant(query: str, collection_name: str) -> str:
    """Search for relevant context in Qdrant using Gemini embeddings."""
    if not qdrant_client:
        return "Error: Qdrant client is not available."
    try:
        embedding = embed_query(query)
        if not embedding:
            return "Error: Could not create an embedding for the query."

        search_response = qdrant_client.search(
            collection_name=collection_name,
            query_vector=embedding,
            limit=3
        )
        context = ""
        for point in search_response:
            context += f"Source: {point.payload.get('metadata', {}).get('source', 'N/A')}, Title: {point.payload.get('metadata', {}).get('title', 'N/A')}\n"
            context += f"Content: {point.payload.get('page_content', '')}\n\n"
        return context
    except Exception as e:
        logging.error(f"Qdrant search failed: {e}")
        return "Error: Could not perform search."

@retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
def get_gemini_chat_response(message: str, preamble: str):
    """Gets a chat response from Gemini, with retry for transient errors."""
    if not gemini_chat_model:
        return "Error: Gemini chat model is not available."
    try:
        # Gemini API uses a different structure, the "preamble" is part of the system instruction
        # or the initial user message. We'll prepend it to the message.
        full_prompt = f"{preamble}\n\n{message}"
        response = gemini_chat_model.generate_content(full_prompt)
        return response.text
    except Exception as e:
        logging.error(f"An unexpected error occurred during chat: {e}")
        # Re-raise for tenacity
        raise

def get_chat_response(message: str, context: str) -> str:
    """Get a response from Gemini based on the context, with subagent routing."""
    
    # 1. Route to subagent if applicable
    subagent = agents.route_to_subagent(message)
    if subagent:
        logging.info(f"Routing to subagent: {subagent.__class__.__name__}")
        return subagent.run(message, context)

    # 2. Proceed with RAG or selected-text logic
    if context:
        logging.info("Using selected-text logic.")
        preamble = "You are a strict question-answering assistant. You must answer the user\'s question based ONLY on the provided text. If the answer is not found in the text, you must respond with 'I don\'t know; the selected text does not contain the answer.'."
        prompt = f"Context:\n{context}\n\nQuestion: {message}"
    else:
        logging.info("No selection context. Using RAG logic.")
        # Fallback for general questions using RAG (no context provided)
        retrieved_context = search_qdrant(message, config.QDRANT_COLLECTION_NAME)
        preamble = "You are a helpful assistant for a digital book. Answer the user\'s question based on the provided context from the book. If the context is not relevant, say that you cannot answer the question with the provided information. Include citations to the source of the information."
        prompt = f"Context:\n{retrieved_context}\n\nQuestion: {message}"
        
    return get_gemini_chat_response(prompt, preamble)