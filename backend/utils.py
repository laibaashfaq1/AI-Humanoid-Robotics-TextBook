import requests
from bs4 import BeautifulSoup
from typing import List, Tuple, Optional, Dict
import logging

logger = logging.getLogger(__name__)

def extract_text_from_url(url: str, selectors: Optional[Dict] = None) -> Tuple[str, str]:
    if selectors is None:
        selectors = {"title": "h1", "content": "main"}

    try:
        response = requests.get(url)
        response.raise_for_status()
        soup = BeautifulSoup(response.content, 'html.parser')

        title_element = soup.select_one(selectors["title"])
        title = title_element.get_text(strip=True) if title_element else "No title found"

        content_element = soup.select_one(selectors["content"])
        if content_element:
            for script_or_style in content_element(["script", "style"]):
                script_or_style.decompose()
            text = content_element.get_text(separator="\n", strip=True)
        else:
            text = "No content found"

        return title, text
    except requests.exceptions.RequestException as e:
        logger.error(f"Error fetching url {url}: {e}")
        return "Error", ""
    except Exception as e:
        logger.error(f"Error extracting text from {url}: {e}")
        return "Error", ""

def chunk_text(text: str, chunk_size: int, chunk_overlap: int) -> List[str]:
    if chunk_size <= 0 or chunk_overlap >= chunk_size:
        raise ValueError("Invalid chunk_size or chunk_overlap")

    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunks.append(text[start:end])
        start += chunk_size - chunk_overlap
    return chunks
