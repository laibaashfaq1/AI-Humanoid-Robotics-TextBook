# Retrieval Pipeline Validation Script

This script validates the integrity and performance of the Qdrant and Cohere-based retrieval pipeline. It offers various checks including basic vector retrieval, semantic search accuracy, and end-to-end data integrity.

## Setup

1.  **Environment Variables**: Create a `.env` file in the `backend/` directory with the following variables:
    ```
    COHERE_API_KEY="your_cohere_api_key"
    QDRANT_API_KEY="your_qdrant_api_key"
    QDRANT_URL="your_qdrant_url"
    ```
    Replace the placeholder values with your actual API keys and Qdrant instance URL.

2.  **Install Dependencies**: Navigate to the `backend/` directory and install the required Python packages using `uv`:
    ```powershell
    cd backend
    uv pip install -r requirements.txt
    ```

3.  **Qdrant Collection Name**:
    Open `backend/validation_script.py` and ensure the `QDRANT_COLLECTION_NAME` constant is set to the correct name of your Qdrant collection.

    ```python
    QDRANT_COLLECTION_NAME = "your_qdrant_collection_name" # e.g., "rag_embedding"
    ```

4.  **Cohere Embedding Model**:
    Verify the `COHERE_EMBEDDING_MODEL` constant in `backend/validation_script.py` matches the model used for embedding your documents.

    ```python
    COHERE_EMBEDDING_MODEL = "embed-english-v3.0" # e.g., "embed-english-light-v3.0"
    ```

5.  **Sample Validation Queries**:
    For semantic search validation, you might want to adjust the `SAMPLE_VALIDATION_QUERIES` in `backend/validation_script.py` to reflect expected queries and results relevant to your dataset.

## Usage

Run the script from the `backend/` directory:

```powershell
python validation_script.py --help
```

### Available Arguments:

*   `--basic-retrieval`: Run basic vector retrieval validation.
*   `--semantic-search`: Run semantic search accuracy validation.
*   `--data-integrity`: Run end-to-end data integrity validation.
*   `--all`: Run all validation checks.
*   `--limit <int>`: Limit for basic retrieval validation (number of vectors). Default is `100`.
*   `--top-k <int>`: Top K results for semantic search validation. Default is `5`.

### Examples:

1.  **Run all checks**:
    ```powershell
    python validation_script.py --all
    ```

2.  **Run only basic retrieval validation with a limit of 50 vectors**:
    ```powershell
    python validation_script.py --basic-retrieval --limit 50
    ```

3.  **Run semantic search validation with top-k set to 10**:
    ```powershall
    python validation_script.py --semantic-search --top-k 10
    ```

4.  **Run data integrity validation**:
    ```powershell
    python validation_script.py --data-integrity
    ```

## Output

The script logs its progress and results to the console. Each validation step will indicate whether it passed or failed, along with relevant details and performance metrics. If any validation fails, the script will exit with a non-zero status code.
