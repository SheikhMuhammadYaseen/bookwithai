import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  const handleSelection = () => {
    const text = window.getSelection().toString();
    if (text) {
      setSelectedText(text);
      setIsOpen(true);
      setMessages([{ role: 'user', content: `Tell me more about: "${text}"` }]);
    }
  };

  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);
  
  const sendMessage = async () => {
      if (!input.trim() && !selectedText.trim()) return;

      const message = input || `Tell me more about: "${selectedText}"`;
      const context = selectedText;

      setMessages(prev => [...prev, { role: 'user', content: message }]);
      setInput('');
      setSelectedText('');

      try {
        const response = await fetch('http://localhost:8000/api/chat', {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify({ message, context }),
        });
        const data = await response.json();
        setMessages(prev => [...prev, { role: 'assistant', content: data.response }]);
      } catch (error) {
        console.error("Error sending message:", error);
        setMessages(prev => [...prev, { role: 'assistant', content: "Sorry, I'm having trouble connecting." }]);
      }
  }

  return (
    <div className={styles.chatContainer}>
      {isOpen && (
        <div className={styles.chatDrawer}>
          <div className={styles.chatHeader}>Chat with the Book</div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`chat-message ${msg.role}`}>
                <p>{msg.content}</p>
              </div>
            ))}
          </div>
          <div className={styles.chatInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question..."
              onKeyDown={(e) => e.key === 'Enter' && sendMessage()}
            />
          </div>
        </div>
      )}
      <button className={styles.chatButton} onClick={() => setIsOpen(!isOpen)}>
        ?
      </button>
    </div>
  );
};

export default Chatbot;
