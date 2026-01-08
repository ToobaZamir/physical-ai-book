import React, { useState } from 'react';

export default function RAGChat() {
  const [query, setQuery] = useState('');
  const [answers, setAnswers] = useState([]);

  const handleAsk = async () => {
    if (!query) return;
    try {
      const res = await fetch(`http://127.0.0.1:8000/ask?query=${encodeURIComponent(query)}`);
      const data = await res.json();
      setAnswers(data.answers || [data.answer]);
    } catch (err) {
      console.error("API Error:", err);
      setAnswers(["Error fetching answer."]);
    }
  };

  return (
    <div style={{ padding: '2rem' }}>
      <h1>RAG Chat</h1>
      <input
        type="text"
        value={query}
        onChange={(e) => setQuery(e.target.value)}
        placeholder="Ask something..."
        style={{ width: '300px', padding: '0.5rem', marginRight: '1rem' }}
      />
      <button onClick={handleAsk} style={{ padding: '0.5rem 1rem' }}>Ask</button>

      <div style={{ marginTop: '2rem' }}>
        {answers.map((ans, i) => (
          <p key={i}>{ans}</p>
        ))}
      </div>
    </div>
  );
}

