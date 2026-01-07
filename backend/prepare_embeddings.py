from pathlib import Path
from openai import OpenAI
import pickle
import os

# Load OpenAI key from environment variable or .env
api_key = os.getenv("OPENAI_API_KEY", "YOUR_OPENAI_KEY")
client = OpenAI(api_key=api_key)

def split_text(text, chunk_size=500):
    words = text.split()
    for i in range(0, len(words), chunk_size):
        yield " ".join(words[i:i+chunk_size])

docs_path = Path("docs")
all_texts = []

for md_file in docs_path.glob("**/*.md"):
    text = md_file.read_text(encoding="utf-8")
    for chunk in split_text(text):
        all_texts.append(chunk)

# Generate embeddings
embeddings = []
for chunk in all_texts:
    emb = client.embeddings.create(
        model="text-embedding-3-large",
        input=chunk
    )["data"][0]["embedding"]
    embeddings.append(emb)

# Save locally
with open("backend/book_embeddings.pkl", "wb") as f:
    pickle.dump({"texts": all_texts, "embeddings": embeddings}, f)

print("Embeddings ready!")
