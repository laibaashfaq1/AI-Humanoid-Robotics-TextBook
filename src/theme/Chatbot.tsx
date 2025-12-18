import React, { useState, useRef, useEffect } from 'react';
import '../css/chatbot.css';

type Message = {
  text: string;
  sender: 'user' | 'bot';
};

const Chatbot = () => {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([
    { text: 'Hello! How can I help you with the AI & Humanoid Robotics book?', sender: 'bot' }
  ]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);

  const endRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    endRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages, loading]);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMsg: Message = { text: input, sender: 'user' };
    setMessages(prev => [...prev, userMsg]);
    setInput('');
    setLoading(true);

    try {
      const res = await fetch('http://127.0.0.1:8000/ask', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ question: userMsg.text }),
      });

      const data = await res.json();

      setMessages(prev => [
        ...prev,
        { text: data.answer || 'No answer found.', sender: 'bot' }
      ]);
    } catch {
      setMessages(prev => [
        ...prev,
        { text: "Sorry, I'm having trouble connecting to the server. Please try again later.", sender: 'bot' }
      ]);
    } finally {
      setLoading(false);
    }
  };

  return (
    <>
      {/* Floating Icon */}
      <div className="chat-fab" onClick={() => setOpen(!open)}>
        💬
      </div>

      {open && (
        <div className="chat-container">
          <div className="chat-header">AI Book Assistant</div>

          <div className="chat-body">
            {messages.map((m, i) => (
              <div key={i} className={`chat-msg ${m.sender}`}>
                {m.text}
              </div>
            ))}

            {loading && <div className="chat-msg bot">Typing...</div>}
            <div ref={endRef} />
          </div>

          <div className="chat-input-area">
            <input
              placeholder="Ask a question..."
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={(e) => e.key === 'Enter' && sendMessage()}
            />
            <button onClick={sendMessage}>➤</button>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;
