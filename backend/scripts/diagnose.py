import asyncio
from qdrant_client import QdrantClient, models
import cohere
import os
import sys

# Add the parent directory to the Python path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from app import config

async def check_cohere():
    """Checks the Cohere API by creating a simple embedding."""
    print("Checking Cohere connectivity...")
    try:
        client = cohere.Client(api_key=config.COHERE_API_KEY)
        response = client.embed(
            texts=["test"],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        embedding = response.embeddings[0]
        print("✅ Cohere API connection successful.")
        return True
    except Exception as e:
        print(f"❌ Cohere API connection failed: {e}")
        return False

async def check_qdrant():
    """Checks the Qdrant service by getting collection info."""
    print("\nChecking Qdrant connectivity...")
    try:
        client = QdrantClient(
            url=config.QDRANT_HOST, 
            api_key=config.QDRANT_API_KEY
        )
        
        # Health check
        # The library doesn't have a simple health check, so we perform a basic operation
        print(f"Checking for collection: '{config.QDRANT_COLLECTION_NAME}'")
        collection_info = client.get_collection(collection_name=config.QDRANT_COLLECTION_NAME)
        
        print(f"✅ Qdrant connection successful.")
        print(f"Collection '{config.QDRANT_COLLECTION_NAME}' info:")
        print(f"  - Vectors count: {collection_info.vectors_count}")
        print(f"  - Points count: {collection_info.points_count}")
        
        return True
    except Exception as e:
        print(f"❌ Qdrant connection failed: {e}")
        return False
        
async def check_postgres():
    """Checks the Postgres connection."""
    print("\nChecking Postgres connectivity...")
    print("Note: No database logic is implemented in the provided backend code, so this check is informational.")
    if config.DATABASE_URL:
        print(f"✅ DATABASE_URL is loaded.")
        # In a real application, you would connect to the database here.
        # Example using psycopg2:
        # try:
        #     conn = psycopg2.connect(config.DATABASE_URL)
        #     print("Postgres connection successful (but no tables checked).")
        #     conn.close()
        # except Exception as e:
        #     print(f"Postgres connection failed: {e}")
    else:
        print("❌ DATABASE_URL is not loaded.")


async def main():
    print("--- Starting diagnostics ---")
    await check_cohere()
    await check_qdrant()
    await check_postgres()
    print("\n--- Diagnostics complete ---")

if __name__ == "__main__":
    asyncio.run(main())
