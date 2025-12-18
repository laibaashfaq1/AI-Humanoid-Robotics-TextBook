import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, Filter, Distance, VectorParams, FieldCondition, MatchValue
import cohere
from typing import List, Dict, Any, Union, Set
from datetime import datetime
import argparse
import logging
import time

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Define a placeholder for RetrievedVector based on the data model
class RetrievedVector:
    def __init__(self, id: str, vector: List[float], payload: Dict[str, Any], score: float = None):
        self.id = id
        self.vector = vector
        self.payload = payload
        self.score = score

class ExpectedResult:
    def __init__(self, book_title: str, source_url: str, min_relevance_score: float = None):
        self.book_title = book_title
        self.source_url = source_url
        self.min_relevance_score = min_relevance_score

class ValidationQuery:
    def __init__(self, query_text: str, expected_results: List[ExpectedResult]):
        self.query_text = query_text
        self.expected_results = expected_results

QDRANT_COLLECTION_NAME = "rag_embedding"  # TODO: Replace with actual collection name
COHERE_EMBEDDING_MODEL = "embed-english-v3.0"

def load_env_variables():
    load_dotenv()
    return {
        "COHERE_API_KEY": os.getenv("COHERE_API_KEY"),
        "QDRANT_API_KEY": os.getenv("QDRANT_API_KEY"),
        "QDRANT_URL": os.getenv("QDRANT_URL"),
    }

def connect_to_qdrant() -> QdrantClient:
    env_vars = load_env_variables()
    qdrant_url = env_vars.get("QDRANT_URL")
    qdrant_api_key = env_vars.get("QDRANT_API_KEY")

    if not qdrant_url or not qdrant_api_key:
        raise ValueError("QDRANT_URL and QDRANT_API_KEY must be set in the .env file")

    client = QdrantClient(host=qdrant_url, api_key=qdrant_api_key)
    return client

def connect_to_cohere() -> cohere.Client:
    env_vars = load_env_variables()
    cohere_api_key = env_vars.get("COHERE_API_KEY")

    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY must be set in the .env file")

    client = cohere.Client(cohere_api_key)
    return client

def check_collection_and_points(client: QdrantClient, collection_name: str) -> bool:
    try:
        if not client.collection_exists(collection_name=collection_name):
            logging.error(f"Error: Qdrant collection '{collection_name}' does not exist.")
            return False
        
        collection_info = client.get_collection(collection_name=collection_name)
        if collection_info.points_count == 0:
            logging.warning(f"Warning: Qdrant collection '{collection_name}' is empty. Some validations may not run or might fail.")
            # For this task, we'll allow continuation but log a warning.
            # Depending on policy, this could be a hard failure.
        return True
    except Exception as e:
        logging.error(f"Error checking Qdrant collection '{collection_name}': {e}")
        return False

def retrieve_random_vectors(client: QdrantClient, collection_name: str, limit: int) -> List[RetrievedVector]:
    try:
        start_time = time.time()
        search_result = client.scroll(
            collection_name=collection_name,
            limit=limit,
            with_vectors=True,
            with_payload=True
        )
        end_time = time.time()
        logging.info(f"Retrieved {len(search_result[0])} random vectors in {end_time - start_time:.4f} seconds.")
        
        retrieved_vectors = []
        for record in search_result[0]:
            retrieved_vectors.append(RetrievedVector(
                id=str(record.id),
                vector=record.vector,
                payload=record.payload
            ))
        return retrieved_vectors
    except Exception as e:
        logging.error(f"Error retrieving random vectors: {e}")
        return []

def validate_vector_retrieval(vectors: List[RetrievedVector], expected_count: int) -> bool:
    if not vectors:
        logging.error("Validation failed: No vectors retrieved.")
        return False

    if len(vectors) != expected_count:
        logging.error(f"Validation failed: Expected {expected_count} vectors, but got {len(vectors)}.")
        return False

    for i, vector in enumerate(vectors):
        if not vector.id:
            logging.error(f"Validation failed: Vector at index {i} has no ID.")
            return False
        if not vector.vector:
            logging.error(f"Validation failed: Vector at index {i} has no vector data.")
            return False
        if not vector.payload:
            logging.error(f"Validation failed: Vector at index {i} has no payload data.")
            return False
        # Optionally, check for specific payload keys
        if "source_url" not in vector.payload or "book_title" not in vector.payload:
            logging.error(f"Validation failed: Vector at index {i} is missing 'source_url' or 'book_title' in payload.")
            return False
            
    logging.info(f"Validation successful: Retrieved {len(vectors)} vectors with expected structure.")
    return True

def run_basic_retrieval_validation(qdrant_client: QdrantClient, collection_name: str, limit: int = 100) -> bool:
    logging.info(f"Running basic vector retrieval validation (fetching {limit} vectors)...")
    start_time = time.time()
    vectors = retrieve_random_vectors(qdrant_client, collection_name, limit)
    result = validate_vector_retrieval(vectors, limit)
    end_time = time.time()
    logging.info(f"Basic retrieval validation finished in {end_time - start_time:.4f} seconds. Status: {'PASS' if result else 'FAIL'}")
    return result

def embed_query(cohere_client: cohere.Client, query_text: str) -> List[float]:
    try:
        start_time = time.time()
        response = cohere_client.embed(
            texts=[query_text],
            model=COHERE_EMBEDDING_MODEL,
            input_type="search_query"  # Crucial for optimal search performance
        )
        end_time = time.time()
        logging.info(f"Embedded query in {end_time - start_time:.4f} seconds.")
        return response.embeddings[0]
    except Exception as e:
        logging.error(f"Error embedding query: {e}")
        return []

def perform_semantic_search(qdrant_client: QdrantClient, collection_name: str, query_vector: List[float], top_k: int) -> List[RetrievedVector]:
    try:
        start_time = time.time()
        search_result = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_vector,
            limit=top_k,
            with_vectors=True,
            with_payload=True
        )
        end_time = time.time()
        logging.info(f"Performed semantic search (top_k={top_k}) in {end_time - start_time:.4f} seconds.")
        
        retrieved_vectors = []
        for record in search_result:
            retrieved_vectors.append(RetrievedVector(
                id=str(record.id),
                vector=record.vector,
                payload=record.payload,
                score=record.score
            ))
        return retrieved_vectors
    except Exception as e:
        logging.error(f"Error performing semantic search: {e}")
        return []

def validate_semantic_search_results(query: ValidationQuery, results: List[RetrievedVector], threshold: float = 0.7) -> bool:
    if not results:
        logging.error(f"Validation failed for query '{query.query_text}': No results retrieved.")
        return False

    all_expected_found = True
    for expected in query.expected_results:
        found_expected = False
        for result in results:
            # Check for book_title and source_url in payload, and relevance score
            if (result.payload.get("book_title") == expected.book_title and
                result.payload.get("source_url") == expected.source_url):
                
                if expected.min_relevance_score is not None and result.score < expected.min_relevance_score:
                    logging.error(f"Validation failed for query '{query.query_text}': Expected result '{expected.book_title}' found, but score {result.score:.2f} is below minimum {expected.min_relevance_score:.2f}.")
                    found_expected = False # Mark as not found if score is too low
                else:
                    found_expected = True
                    break
        
        if not found_expected:
            all_expected_found = False
            logging.error(f"Validation failed for query '{query.query_text}': Expected result '{expected.book_title}' (URL: {expected.source_url}) not found in top results or score too low.")
    
    if all_expected_found:
        logging.info(f"Validation successful for query '{query.query_text}': All expected results found with sufficient relevance.")
    
    return all_expected_found

# Define sample validation queries
SAMPLE_VALIDATION_QUERIES: List[ValidationQuery] = [
    ValidationQuery(
        query_text="robot arm control in industrial settings",
        expected_results=[
            ExpectedResult(
                book_title="Introduction to Robotics",
                source_url="https://example.com/books/robotics-intro",
                min_relevance_score=0.75
            ),
            ExpectedResult(
                book_title="Industrial Automation Handbook",
                source_url="https://example.com/books/automation-handbook",
                min_relevance_score=0.70
            )
        ]
    ),
    ValidationQuery(
        query_text="history of artificial intelligence",
        expected_results=[
            ExpectedResult(
                book_title="AI: A Modern Approach",
                source_url="https://example.com/books/ai-modern-approach",
                min_relevance_score=0.80
            )
        ]
    )
]

def run_semantic_search_validation(
    qdrant_client: QdrantClient, 
    cohere_client: cohere.Client, 
    collection_name: str, 
    queries: List[ValidationQuery], 
    top_k: int = 5
) -> bool:
    logging.info(f"Running semantic search validation with {len(queries)} queries (top_k={top_k})...")
    start_time = time.time()
    all_queries_passed = True
    for i, query in enumerate(queries):
        logging.info(f"\n--- Query {i+1}/{len(queries)}: '{query.query_text}' ---")
        query_vector = embed_query(cohere_client, query.query_text)
        if not query_vector:
            logging.error(f"Skipping query '{query.query_text}' due to embedding failure.")
            all_queries_passed = False
            continue

        results = perform_semantic_search(qdrant_client, collection_name, query_vector, top_k)
        if not validate_semantic_search_results(query, results):
            all_queries_passed = False
    
    end_time = time.time()
    if all_queries_passed:
        logging.info(f"\nAll semantic search validation queries passed in {end_time - start_time:.4f} seconds.")
    else:
        logging.error(f"\nSome semantic search validation queries failed. Total time: {end_time - start_time:.4f} seconds.")
    
    return all_queries_passed

def get_all_source_urls_from_qdrant(client: QdrantClient, collection_name: str) -> Set[str]:
    source_urls = set()
    offset = None
    while True:
        try:
            scroll_start_time = time.time()
            scroll_result, next_page_offset = client.scroll(
                collection_name=collection_name,
                limit=100,  # Fetch in batches
                offset=offset,
                with_payload=True,
                with_vectors=False,
                # Set specific payload fields to retrieve, to reduce network traffic
                payload_selector=["source_url"]
            )
            scroll_end_time = time.time()
            logging.info(f"Scrolled {len(scroll_result)} records in {scroll_end_time - scroll_start_time:.4f} seconds.")

            for record in scroll_result:
                if record.payload and "source_url" in record.payload:
                    source_urls.add(record.payload["source_url"])
            
            if next_page_offset is None:
                break
            offset = next_page_offset
        except Exception as e:
            logging.error(f"Error getting all source URLs from Qdrant: {e}")
            break
    return source_urls

def validate_data_integrity(client: QdrantClient, cohere_client: cohere.Client, collection_name: str, source_urls: Set[str]) -> bool:
    logging.info(f"Running data integrity validation for {len(source_urls)} unique source URLs...")
    start_time = time.time()
    all_integrity_checks_passed = True
    
    cohere_model_dimensions = {
        "embed-english-v3.0": 1024,
        "embed-english-light-v3.0": 384, # Example dimension, adjust if different
        # Add other Cohere models and their dimensions as needed
    }
    expected_vector_dimension = cohere_model_dimensions.get(COHERE_EMBEDDING_MODEL)

    if not expected_vector_dimension:
        logging.warning(f"Warning: Unknown Cohere model '{COHERE_EMBEDDING_MODEL}'. Cannot validate vector dimensions.")
        # Continue without dimension validation, or fail based on policy
        # For now, we'll continue but this is a potential point of failure
        pass

    for i, url in enumerate(source_urls):
        logging.info(f"\n--- Checking integrity for URL {i+1}/{len(source_urls)}: {url} ---")
        
        try:
            scroll_result, _ = client.scroll(
                collection_name=collection_name,
                query_filter=Filter(
                    must=[
                        FieldCondition(
                            key="source_url",
                            match=MatchValue(value=url)
                        )
                    ]
                ),
                limit=100, # Assuming max 100 chunks per URL for efficiency; adjust if needed
                with_vectors=True,
                with_payload=True
            )
            
            if not scroll_result:
                logging.error(f"Integrity check failed: No vectors found for URL: {url}")
                all_integrity_checks_passed = False
                continue

            url_integrity_passed = True
            for record in scroll_result:
                vector = RetrievedVector(
                    id=str(record.id),
                    vector=record.vector,
                    payload=record.payload
                )

                # Validate metadata presence
                required_payload_keys = ["source_url", "chunk_id", "book_title", "timestamp"]
                for key in required_payload_keys:
                    if key not in vector.payload:
                        logging.error(f"Integrity check failed: Vector {vector.id} (URL: {url}) missing required payload key: '{key}'.")
                        url_integrity_passed = False
                
                # Validate vector dimensions
                if expected_vector_dimension and vector.vector and len(vector.vector) != expected_vector_dimension:
                    logging.error(f"Integrity check failed: Vector {vector.id} (URL: {url}) has dimension {len(vector.vector)}, expected {expected_vector_dimension}.")
                    url_integrity_passed = False

                # Validate timestamp format (basic check)
                if "timestamp" in vector.payload:
                    try:
                        datetime.fromisoformat(vector.payload["timestamp"])
                    except ValueError:
                        logging.error(f"Integrity check failed: Vector {vector.id} (URL: {url}) has invalid timestamp format: {vector.payload['timestamp']}.")
                        url_integrity_passed = False
                        
            if url_integrity_passed:
                logging.info(f"Integrity check passed for URL: {url}")
            else:
                all_integrity_checks_passed = False

        except Exception as e:
            logging.error(f"Error during integrity check for URL {url}: {e}")
            all_integrity_checks_passed = False
            
    end_time = time.time()
    if all_integrity_checks_passed:
        logging.info(f"\nAll data integrity validation checks passed in {end_time - start_time:.4f} seconds.")
    else:
        logging.error(f"\nSome data integrity validation checks failed. Total time: {end_time - start_time:.4f} seconds.")
    
    return all_integrity_checks_passed

def run_data_integrity_validation(qdrant_client: QdrantClient, cohere_client: cohere.Client, collection_name: str) -> bool:
    logging.info("Running data integrity validation...")
    start_time = time.time()
    source_urls = get_all_source_urls_from_qdrant(qdrant_client, collection_name)
    result = validate_data_integrity(qdrant_client, cohere_client, collection_name, source_urls)
    end_time = time.time()
    logging.info(f"Data integrity validation finished in {end_time - start_time:.4f} seconds. Status: {'PASS' if result else 'FAIL'}")
    return result


def main():
    parser = argparse.ArgumentParser(description="Validate Qdrant and Cohere Retrieval Pipeline.")
    parser.add_argument("--basic-retrieval", action="store_true", help="Run basic vector retrieval validation.")
    parser.add_argument("--semantic-search", action="store_true", help="Run semantic search accuracy validation.")
    parser.add_argument("--data-integrity", action="store_true", help="Run end-to-end data integrity validation.")
    parser.add_argument("--all", action="store_true", help="Run all validation checks.")
    parser.add_argument("--limit", type=int, default=100, help="Limit for basic retrieval validation (number of vectors).")
    parser.add_argument("--top-k", type=int, default=5, help="Top K results for semantic search validation.")

    args = parser.parse_args()

    # Handle error for invalid/missing environment variables
    try:
        env_vars = load_env_variables()
        qdrant_client = connect_to_qdrant()
        cohere_client = connect_to_cohere()
    except ValueError as e:
        logging.error(f"Environment variable error: {e}")
        exit(1)
    
    if not check_collection_and_points(qdrant_client, QDRANT_COLLECTION_NAME):
        logging.error(f"Qdrant collection '{QDRANT_COLLECTION_NAME}' is not ready or accessible. Exiting.")
        exit(1)

    overall_status = True

    if args.basic_retrieval or args.all:
        logging.info("\n--- Starting Basic Retrieval Validation ---")
        if not run_basic_retrieval_validation(qdrant_client, QDRANT_COLLECTION_NAME, args.limit):
            overall_status = False
        logging.info("--- Basic Retrieval Validation Finished ---\\n")

    if args.semantic_search or args.all:
        logging.info("\n--- Starting Semantic Search Validation ---")
        if not run_semantic_search_validation(qdrant_client, cohere_client, QDRANT_COLLECTION_NAME, SAMPLE_VALIDATION_QUERIES, args.top_k):
            overall_status = False
        logging.info("--- Semantic Search Validation Finished ---\\n")

    if args.data_integrity or args.all:
        logging.info("\n--- Starting Data Integrity Validation ---")
        if not run_data_integrity_validation(qdrant_client, cohere_client, QDRANT_COLLECTION_NAME):
            overall_status = False
        logging.info("--- Data Integrity Validation Finished ---\\n")
    
    if not (args.basic_retrieval or args.semantic_search or args.data_integrity or args.all):
        logging.info("No validation checks specified. Use --help for options.")
    
    if overall_status:
        logging.info("All selected validation checks passed successfully!")
        exit(0)
    else:
        logging.error("One or more validation checks failed.")
        exit(1)

if __name__ == "__main__":
    main()