import requests

def test_ask_endpoint(question: str):
    """
    Tests the /ask endpoint of the FastAPI application.
    """
    url = "http://127.0.0.1:8000/ask"
    payload = {"question": question}
    
    try:
        response = requests.post(url, json=payload)
        response.raise_for_status()  # Raise an exception for bad status codes
        
        data = response.json()
        print("Response from /ask endpoint:")
        print(data)
        
    except requests.exceptions.RequestException as e:
        print(f"Error calling the /ask endpoint: {e}")

if __name__ == "__main__":
    test_question = "What is ROS 2?"
    test_ask_endpoint(test_question)