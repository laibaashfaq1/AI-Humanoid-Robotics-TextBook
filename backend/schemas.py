from typing import List, Optional, Dict, Any
from pydantic import BaseModel

class RetrievedDocumentChunk(BaseModel):
    id: str
    text: str
    metadata: Dict[str, Any]
    score: float

class AskRequest(BaseModel):
    question: str

class AskResponse(BaseModel):
    answer: str
    retrieved_chunks: Optional[List[RetrievedDocumentChunk]] = []
