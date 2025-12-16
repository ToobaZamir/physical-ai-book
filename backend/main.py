from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os
from dotenv import load_dotenv

load_dotenv()

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

@app.get("/")
def read_root():
    return {"message": "RAG API Running Successfully!"}

@app.get("/ask")
def ask_question(query: str):
    # Ye placeholder hai, abhi OpenAI + Qdrant integration add karna hai
    return {"question": query, "answer": "This will be answered by RAG model"}







#
from fastapi import FastAPI
from qdrant_client import QdrantClient

app = FastAPI()

client = QdrantClient(
    url="https://ac177d4f-2fa2-4031-bc22-d90a5b5e05cb.europe-west3-0.gcp.cloud.qdrant.io",
    api_key="YOUR_API_KEY_HERE"
)

@app.get("/")
def health():
    return {"status": "Qdrant connected!"}





#
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def home():
    return {"message": "RAG API Running Successfully!"}
