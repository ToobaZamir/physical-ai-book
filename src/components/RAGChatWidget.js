import React, { useState } from "react";
import "./rag.css";

const RAGChatWidget = () => {
  const [query, setQuery] = useState("");
  const [answers, setAnswers] = useState([]);
  const [loading, setLoading] = useState(false);

  const askQuestion = async () => {
    if (!query) return;
    setLoading(true);
    try {
      const res = await fetch(`http://127.0.0.1:8000/ask?query=${encodeURIComponent(query)}`);
      const data = await res.json();
      setAnswers(data.answers || []);
    } catch (err) {
      console.error(err);
      setAnswers(["Error connecting to backend"]);
    }
    setLoading(false);
  };

  return (
    <div className="rag-widget">
      <h3>RAG Chat</h3>
      <input
        type="text"
        value={query}
        onChange={(e) => setQuery(e.target.value)}
        placeholder="Ask a question..."
      />
      <button onClick={askQuestion} disabled={loading}>
        {loading ? "Loading..." : "Ask"}
      </button>
      <div className="answers">
        {answers.map((ans, idx) => (
          <p key={idx}>{ans}</p>
        ))}
      </div>
    </div>
  );
};

export default RAGChatWidget;
