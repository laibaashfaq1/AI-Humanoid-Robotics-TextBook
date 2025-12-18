import os
import requests
import uuid
import datetime
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET
from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient, models
from langchain_text_splitters import RecursiveCharacterTextSplitter
import logging

# --- Configuration ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Load environment variables from .env file
load_dotenv()

# --- Constants ---
BOOK_SITEMAP_URL = "https://ai-humanoid-robotics-text-book.vercel.app/sitemap.xml"
COHERE_MODEL_NAME = "embed-english-light-v3.0"
COHERE_INPUT_TYPE = "search_document"
QDRANT_COLLECTION_NAME = "rag_embedding"

# Vector size for the specified Cohere model
# embed-english-light-v3.0 -> 384
# You may need to change this if you change the model
COHERE_VECTOR_SIZE = 384

# --- Initialize Clients ---
try:
    cohere_api_key = os.environ["COHERE_API_KEY"]
    qdrant_url = os.environ["QDRANT_URL"]
    qdrant_api_key = os.environ.get("QDRANT_API_KEY") # API key is optional for local Qdrant

    co = cohere.Client(cohere_api_key)
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
except KeyError as e:
    logging.error(f"FATAL: Environment variable {e} not set. Please create a .env file with COHERE_API_KEY, QDRANT_URL, and optionally QDRANT_API_KEY.")
    exit(1)


def get_all_urls_from_sitemap(sitemap_url: str) -> list[str]:
    """
    Crawls a sitemap.xml file to extract all URLs.
    """
    logging.info(f"Crawling sitemap: {sitemap_url}")
    try:
        response = requests.get(sitemap_url)
        response.raise_for_status()

        root = ET.fromstring(response.content)
        urls = [
            elem.text
            for elem in root.findall(".//{http://www.sitemaps.org/schemas/sitemap/0.9}loc")
            if elem.text
        ]
        logging.info(f"Found {len(urls)} URLs in sitemap.")
        return urls
    except requests.exceptions.RequestException as e:
        logging.error(f"Error fetching sitemap: {e}")
        return []
    except ET.ParseError as e:
        logging.error(f"Error parsing sitemap XML: {e}")
        return []

def scrape_page_content(url: str) -> dict | None:
    """
    Scrapes a given URL to extract the page title and clean, readable text
    from the main content area, ignoring navs, footers, etc.
    """
    try:
        response = requests.get(url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, "html.parser")

        # The book's main content is within the <main> tag
        main_content = soup.find("main")
        if not main_content:
            logging.warning(f"No <main> tag found on {url}, skipping.")
            return None

        # Extract text and clean it up
        text = main_content.get_text(separator="\n", strip=True)
        page_title = soup.title.string if soup.title else "No Title"

        return {"text": text, "page_title": page_title, "source_url": url}
    except requests.exceptions.RequestException as e:
        logging.error(f"Could not retrieve or read page {url}: {e}")
        return None

def main():
    """
    Main orchestration function to crawl, process, and ingest the book content.
    """
    logging.info("Starting book ingestion process...")

    # 1. Ensure Qdrant collection exists
    try:
        qdrant_client.get_collection(collection_name=QDRANT_COLLECTION_NAME)
        logging.info(f"Qdrant collection '{QDRANT_COLLECTION_NAME}' already exists.")
    except Exception:
        logging.info(f"Collection '{QDRANT_COLLECTION_NAME}' not found. Creating new collection.")
        qdrant_client.recreate_collection(
            collection_name=QDRANT_COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=COHERE_VECTOR_SIZE,
                distance=models.Distance.COSINE,
            ),
        )
        logging.info(f"Collection '{QDRANT_COLLECTION_NAME}' created successfully.")

    # 2. Crawl sitemap
    urls = get_all_urls_from_sitemap(BOOK_SITEMAP_URL)
    if not urls:
        logging.error("No URLs found. Exiting.")
        return

    # 3. For each page: scrape, clean, and chunk
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=500,
        chunk_overlap=50,
        length_function=len,
    )

    all_chunks = []
    logging.info("Scraping and chunking all pages...")
    for i, url in enumerate(urls):
        logging.info(f"Processing page {i+1}/{len(urls)}: {url}")
        page_data = scrape_page_content(url)
        if page_data:
            chunks = text_splitter.split_text(page_data["text"])
            for chunk_index, chunk_text in enumerate(chunks):
                all_chunks.append({
                    "text": chunk_text,
                    "source_url": page_data["source_url"],
                    "page_title": page_data["page_title"],
                    "chunk_id": chunk_index
                })

    logging.info(f"Total chunks created: {len(all_chunks)}")
    if not all_chunks:
        logging.error("No text chunks were generated. Exiting.")
        return

    # 4. Generate embeddings using Cohere
    logging.info("Generating embeddings with Cohere...")
    chunk_texts_for_embedding = [chunk['text'] for chunk in all_chunks]
    
    try:
        response = co.embed(
            texts=chunk_texts_for_embedding,
            model=COHERE_MODEL_NAME,
            input_type=COHERE_INPUT_TYPE,
        )
        embeddings = response.embeddings
        logging.info(f"Successfully generated {len(embeddings)} embeddings.")
    except cohere.CohereError as e:
        logging.error(f"Cohere API error: {e}")
        return

    # 5. Insert embeddings into Qdrant
    logging.info("Preparing and upserting points to Qdrant...")
    now_timestamp = datetime.datetime.now(datetime.timezone.utc).isoformat()
    
    points_to_upsert = [
        models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "text": chunk["text"],
                "source_url": chunk["source_url"],
                "page_title": chunk["page_title"],
                "chunk_id": chunk["chunk_id"],
                "timestamp": now_timestamp,
            },
        )
        for chunk, embedding in zip(all_chunks, embeddings)
    ]

    # Upsert in batches for efficiency
    qdrant_client.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        points=points_to_upsert,
        wait=True,
    )

    logging.info(f"Successfully upserted {len(points_to_upsert)} points into Qdrant collection '{QDRANT_COLLECTION_NAME}'.")
    logging.info("Ingestion process complete.")


if __name__ == "__main__":
    main()
