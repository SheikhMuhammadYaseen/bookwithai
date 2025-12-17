import os
from dotenv import load_dotenv

# Load .env from project root
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "..", "..", ".env"))

# OpenAI API key
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

# Cohere API key
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

# Qdrant
QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")
