from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv
from cohere import Client

load_dotenv()
cohere_client = Client(os.getenv("COHERE_API_KEY"))


app = FastAPI(title="Physical AI RAG API", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)


@app.get("/ask")
def ask_question(query: str):
    # 1️⃣ Query ko embedding me convert karo
    embedding = cohere_client.embed(texts=[query], model="small").embeddings[0]

    # 2️⃣ Qdrant me similar documents search karo
    results = client.search_points(
        collection_name="book",  # apni collection ka naam
        query_vector=embedding,
        limit=3
    )

    # 3️⃣ Relevant text nikal lo
    answers = [hit.payload['text'] for hit in results]

    return {"question": query, "answers": answers}



@app.get("/")
def health():
    return {"status": "RAG API Running Successfully!", "qdrant": "Connected"}


