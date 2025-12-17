import os
import sys
import markdown
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from langchain_qdrant import QdrantVectorStore  # New import
from langchain_cohere import CohereEmbeddings
from langchain_core.documents import Document


def main():
    # Load .env
    dotenv_path = os.path.join(os.path.dirname(__file__), "..", "..", ".env")
    load_dotenv(dotenv_path=dotenv_path)

    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_host = os.getenv("QDRANT_HOST")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")
    collection_name = os.getenv("QDRANT_COLLECTION_NAME", "rag_embedding")

    if not all([cohere_api_key, qdrant_host, qdrant_api_key]):
        print("Error: Missing required env vars (COHERE_API_KEY, QDRANT_HOST, QDRANT_API_KEY)")
        sys.exit(1)

    embeddings = CohereEmbeddings(
        cohere_api_key=cohere_api_key,
        model="embed-english-v3.0"
    )

    # Local docs folder
    docs_folder = os.path.join(os.path.dirname(__file__), "..", "..", "digital-book", "docs")
    if not os.path.exists(docs_folder):
        print(f"Error: Docs folder not found: {docs_folder}")
        sys.exit(1)

    md_files = []
    for root, _, files in os.walk(docs_folder):
        for file in files:
            if file.endswith(".md") or file.endswith(".mdx"):
                md_files.append(os.path.join(root, file))

    if not md_files:
        print("No Markdown files found!")
        sys.exit(1)

    print(f"Found {len(md_files)} files. Processing...")

    all_docs = []
    for i, file_path in enumerate(md_files, 1):
        print(f"[{i}/{len(md_files)}] {file_path}")
        text = extract_text_from_md(file_path)
        if text:
            chunks = chunk_text(text, chunk_size=800, overlap=100)
            relative_path = os.path.relpath(file_path, docs_folder)
            for chunk in chunks:
                all_docs.append(Document(
                    page_content=chunk,
                    metadata={"source": relative_path}
                ))

    if not all_docs:
        print("No documents created!")
        sys.exit(1)

    print(f"Uploading {len(all_docs)} chunks to Qdrant...")
    try:
        QdrantVectorStore.from_documents(
            documents=all_docs,
            embedding=embeddings,
            url=qdrant_host,
            api_key=qdrant_api_key,
            collection_name=collection_name,
            force_recreate=True,  # Old collection delete karega
        )
        print("Population successful! Collection ready.")
    except Exception as e:
        print(f"Upload failed: {e}")
        sys.exit(1)


def extract_text_from_md(file_path: str):
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            md_content = f.read()
        html = markdown.markdown(md_content)
        soup = BeautifulSoup(html, 'html.parser')
        text = soup.get_text(separator=' ', strip=True)
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        clean_text = ' '.join(chunk for chunk in chunks if chunk)
        return clean_text if len(clean_text) > 50 else None
    except Exception as e:
        print(f"Error reading {file_path}: {e}")
        return None


def chunk_text(text: str, chunk_size: int = 800, overlap: int = 100):
    if len(text) <= chunk_size:
        return [text]
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start = end - overlap
    return chunks


if __name__ == "__main__":
    main()