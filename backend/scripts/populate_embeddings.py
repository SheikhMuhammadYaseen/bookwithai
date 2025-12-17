
import os
import cohere
from dotenv import load_dotenv
from qdrant_client import QdrantClient, models
from unstructured.chunking.title import chunk_by_title
from unstructured.client import UnstructuredClient
from unstructured.documents.elements import CompositeElement
from unstructured.partition.md import partition_md

# Load environment variables from .env file
load_dotenv()

# Initialize Cohere client
cohere_api_key = os.getenv("COHERE_API_KEY")
co = cohere.Client(cohere_api_key)

# Initialize Qdrant client
qdrant_url = os.getenv("QDRANT_URL")
qdrant_api_key = os.getenv("QDRANT_API_KEY")
qdrant_client = QdrantClient(
    url=qdrant_url,
    api_key=qdrant_api_key,
)

# Initialize Unstructured client
unstructured_api_key = os.getenv("UNSTRUCTURED_API_KEY")
s = UnstructuredClient(api_key=unstructured_api_key)


def get_doc_layouts(filenames):
    """
    Partitions the documents into sections and groups them by document.
    """
    doc_layouts = {}
    for fn in filenames:
        print(f"Processing {fn}...")
        layouts = partition_md(filename=fn, include_metadata=False)
        doc_layouts[fn] = layouts
    return doc_layouts


def chunk_layouts(doc_layouts):
    """
    Chunks the layouts into smaller pieces for embedding.
    """
    doc_chunks = {}
    for fn, layout in doc_layouts.items():
        chunks = chunk_by_title(
            layout, combine_text_under_n_chars=0, new_after_n_chars=1024
        )
        doc_chunks[fn] = chunks
    return doc_chunks


def main():
    """
    Main function to run the embedding pipeline.
    """
    # Find all markdown files in the digital-book/docs directory
    docs_path = "../../digital-book/docs"
    filenames = []
    for root, dirs, files in os.walk(docs_path):
        for file in files:
            if file.endswith(".md"):
                filenames.append(os.path.join(root, file))

    # Partition and chunk the documents
    doc_layouts = get_doc_layouts(filenames)
    doc_chunks = chunk_layouts(doc_layouts)

    # Create a Qdrant collection
    collection_name = "docusaurus_embeddings"
    try:
        qdrant_client.recreate_collection(
            collection_name=collection_name,
            vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),
        )
        print(f"Collection '{collection_name}' created.")
    except Exception as e:
        print(f"Collection already exists or error: {e}")

    # Generate and upload embeddings
    points = []
    for fn, chunks in doc_chunks.items():
        for i, chunk in enumerate(chunks):
            if isinstance(chunk, CompositeElement):
                text = chunk.text
                response = co.embed(
                    texts=[text], model="embed-english-v3.0", input_type="search_document"
                )
                embedding = response.embeddings[0]
                point = models.PointStruct(
                    id=f"{fn}-{i}",
                    vector=embedding,
                    payload={"text": text, "source": fn},
                )
                points.append(point)

    if points:
        qdrant_client.upsert(
            collection_name=collection_name,
            points=points,
            wait=True,
        )
        print(f"Upserted {len(points)} points to collection '{collection_name}'.")


if __name__ == "__main__":
    main()
