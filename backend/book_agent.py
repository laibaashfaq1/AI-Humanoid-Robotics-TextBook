# book_agent.py
import os
import openai
from dotenv import load_dotenv
from typing import Any

load_dotenv()

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")
openai.api_key = OPENAI_API_KEY

class BookAgent:
    def __init__(self):
        pass  # No agent initialization needed

    def chat(self, full_query_for_agent: str) -> Any:
        """
        Sends the user question + context to OpenAI GPT and returns the response.
        """
        try:
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[{"role": "user", "content": full_query_for_agent}],
                temperature=0
            )
            answer_text = response.choices[0].message["content"]
            return answer_text
        except Exception as e:
            print(f"Error calling OpenAI API: {e}")
            return "Sorry, I could not process your question at this time."
