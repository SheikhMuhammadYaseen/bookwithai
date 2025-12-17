import os
from dotenv import load_dotenv

<<<<<<< HEAD
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
=======
load_dotenv(dotenv_path=".env")

GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")
GEMINI_EMBED_MODEL = os.getenv("GEMINI_EMBED_MODEL", "text-embedding-gecko-001")
GEMINI_CHAT_MODEL = os.getenv("GEMINI_CHAT_MODEL", "gemini-pro")

QDRANT_HOST = os.getenv("QDRANT_HOST")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_COLLECTION_name = os.getenv("QDRANT_COLLECTION_NAME", "digital-book-collection")
DATABASE_URL = os.getenv("DATABASE_URL")
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
