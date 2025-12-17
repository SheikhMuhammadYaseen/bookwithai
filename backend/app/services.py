import logging
from qdrant_client import QdrantClient
from langchain_qdrant import QdrantVectorStore
from langchain_cohere import CohereEmbeddings, ChatCohere
from langchain_core.messages import HumanMessage, SystemMessage
from . import config, agents


# Qdrant client
qdrant_client = QdrantClient(
    url=config.QDRANT_HOST,
    api_key=config.QDRANT_API_KEY
)

# Embeddings
embeddings = CohereEmbeddings(
    cohere_api_key=config.COHERE_API_KEY,
    model="embed-english-v3.0"
)

# Chat model
chat_model = ChatCohere(
    cohere_api_key=config.COHERE_API_KEY,
    model="command-a-03-2025",
    temperature=0.7
)

def search_qdrant(query: str, collection_name: str) -> str:
    try:
        vectorstore = QdrantVectorStore(
            client=qdrant_client,
            collection_name=collection_name,
            embedding=embeddings
        )
        results = vectorstore.similarity_search(query, k=8)
        
        context = ""
        for doc in results:
            text = doc.page_content
            source = doc.metadata.get("source", "Unknown")
            context += f"{text}\n\n" 
        
        return context or "No context found."
    except Exception as e:
        logging.error(f"Search error: {e}")
        return ""

def get_chat_response(message: str, context: str = None) -> str:
    subagent = agents.route_to_subagent(message) if hasattr(agents, 'route_to_subagent') else None
    if subagent:
        return subagent.run(message, context)

    if not context:
        context = search_qdrant(message, config.QDRANT_COLLECTION_NAME)

    if not context.strip():
        return "Hmm, I couldn't find anything specific in the book about that. Try selecting a different part or asking directly!"

    system_prompt = f"""
You are a friendly assistant helping with content from the book.

Answer the question using the context below. Make it natural and easy to read.

Use bullet points for lists if needed.
Use **bold** or *italics* lightly for key terms.

Context:
{context}

Question: {message}

Try to give a helpful answer even if the context is partial.
"""

    try:
        messages = [
            SystemMessage(content=system_prompt),
            HumanMessage(content=message)
        ]
        response = chat_model.invoke(messages)
        return response.content.strip()
    except Exception as e:
        logging.error(f"Chat error: {e}")
        return "Sorry, I'm having trouble connecting right now. Try again!"

def personalize_content(
    role: str,
    skill_level: str,
    programming_language: str,
    book_content: str,
    query: str,
) -> str:
    """
    Invokes the PersonalizationAgent to get a tailored response.
    """
    try:
        agent = agents.PersonalizationAgent()
        response = agent.run(
            role=role,
            skill_level=skill_level,
            programming_language=programming_language,
            book_content=book_content,
            query=query,
        )
        return response
    except Exception as e:
        logging.error(f"Personalization error: {e}")
        return "Sorry, I encountered an error during personalization. Please try again."
