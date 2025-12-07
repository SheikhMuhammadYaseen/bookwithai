import os
import argparse
from dotenv import load_dotenv
from langchain.text_splitter import MarkdownHeaderTextSplitter
from langchain_cohere import CohereEmbeddings # Changed from OpenAIEmbeddings
from langchain.vectorstores import Qdrant
from unstructured.chunking.title import chunk_by_title
from unstructured.partition.md import partition_md
from unstructured.documents.elements import Title

def main(docs_path):
    load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), '..', '.env'))

    # Check for Cohere API key
    if not os.getenv("COHERE_API_KEY"):
        raise ValueError("COHERE_API_KEY not found in environment variables.")

    # Get Qdrant config
    qdrant_host = os.getenv("QDRANT_HOST")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    qdrant_collection_name = os.getenv("QDRANT_COLLECTION_NAME")

    if not all([qdrant_host, qdrant_api_key, qdrant_collection_name]):
        raise ValueError("Qdrant configuration not found in environment variables.")
    
    print(f"Processing documents in: {docs_path}")
    
    # Process each markdown file
    all_chunks = []
    for root, _, files in os.walk(docs_path):
        for file in files:
            if file.endswith(".md"):
                file_path = os.path.join(root, file)
                print(f"Processing {file_path}")
                
                elements = partition_md(filename=file_path)
                
                # Find the main title of the document
                doc_title = file 
                for element in elements:
                    if isinstance(element, Title):
                        doc_title = element.text
                        break

                chunks = chunk_by_title(elements)
                
                for chunk in chunks:
                    chunk.metadata.source = os.path.relpath(file_path, docs_path)
                    chunk.metadata.title = doc_title
                    all_chunks.append(chunk)

    print(f"Generated {len(all_chunks)} chunks.")

    # Generate embeddings and upload to Qdrant
    embeddings = CohereEmbeddings(cohere_api_key=os.getenv("COHERE_API_KEY"), model="embed-english-v3.0")

    print("Uploading to Qdrant...")
    Qdrant.from_documents(
        all_chunks,
        embeddings,
        url=qdrant_host,
        api_key=qdrant_api_key,
        collection_name=qdrant_collection_name,
    )
    print("Population complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Populate Qdrant with markdown content.")
    parser.add_argument("--docs-path", type=str, required=True, help="Path to the directory containing markdown files.")
    args = parser.parse_args()
    main(args.docs_path)
