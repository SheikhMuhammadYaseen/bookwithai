<<<<<<< HEAD
import React, { useState, useEffect, useRef } from 'react';
=======
import React, { useState, useEffect } from 'react';
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
import styles from './styles.module.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');
<<<<<<< HEAD
  const [messages, setMessages] = useState<Array<{ role: string; content: string }>>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [hasInitialSelectionMessage, setHasInitialSelectionMessage] = useState(false);

  const messagesEndRef = useRef<HTMLDivElement>(null);
  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSelection = () => {
    const text = window.getSelection()?.toString().trim();
    if (text) {
      setSelectedText(text);
      setIsOpen(true);

      if (!hasInitialSelectionMessage) {
        const initialMessage = `Tell me more about: "${text}"`;
        setMessages([{ role: 'user', content: initialMessage }]);
        setHasInitialSelectionMessage(true);
      }
=======
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');

  const handleSelection = () => {
    const text = window.getSelection().toString();
    if (text) {
      setSelectedText(text);
      setIsOpen(true);
      setMessages([{ role: 'user', content: `Tell me more about: "${text}"` }]);
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
    }
  };

  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
<<<<<<< HEAD
  }, [hasInitialSelectionMessage]);

  const sendMessage = async () => {
    if (!input.trim() && !selectedText.trim()) return;

    let message: string;
    let context: string = selectedText;

    if (input.trim()) {
      message = input;
    } else {
      message = messages.find(m => m.role === 'user')?.content || `Tell me more about: "${selectedText}"`;
    }

    if (input.trim()) {
      setMessages(prev => [...prev, { role: 'user', content: message }]);
    }

    setMessages(prev => [...prev, { role: 'assistant', content: 'thinking' }]);
    setIsLoading(true);

    setInput('');
    setSelectedText('');

    try {
      const response = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message, context }),
      });
      const data = await response.json();

      setMessages(prev => {
        const newMessages = [...prev];
        newMessages[newMessages.length - 1] = { role: 'assistant', content: data.response };
        return newMessages;
      });
    } catch (error) {
      console.error("Error sending message:", error);
      setMessages(prev => {
        const newMessages = [...prev];
        newMessages[newMessages.length - 1] = { role: 'assistant', content: "Sorry, I'm having trouble connecting." };
        return newMessages;
      });
    } finally {
      setIsLoading(false);
    }
  };
=======
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
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac

  return (
    <div className={styles.chatContainer}>
      {isOpen && (
        <div className={styles.chatDrawer}>
<<<<<<< HEAD
          <div className={styles.chatHeader}>
            Chat with the Book
            <button className={styles.closeButton} onClick={() => setIsOpen(false)}>
              ×
            </button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.chatMessage} ${styles[msg.role]}`}>
                {msg.content === 'thinking' ? (
                  <div className={styles.thinking}>
                    <span>Thinking</span>
                    <span className={styles.dot1}>.</span>
                    <span className={styles.dot2}>.</span>
                    <span className={styles.dot3}>.</span>
                  </div>
                ) : (
                  <p>{msg.content}</p>
                )}
              </div>
            ))}
            <div ref={messagesEndRef} />
=======
          <div className={styles.chatHeader}>Chat with the Book</div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`chat-message ${msg.role}`}>
                <p>{msg.content}</p>
              </div>
            ))}
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
          </div>
          <div className={styles.chatInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
<<<<<<< HEAD
              placeholder="Ask a question or follow up..."
              onKeyDown={(e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                  e.preventDefault();
                  sendMessage();
                }
              }}
              rows={3}
            />
            <button onClick={sendMessage} disabled={isLoading}>
              Send
            </button>
          </div>
        </div>
      )}
      {/* Chat button with full-logo */}
      <button className={styles.chatButton} onClick={() => setIsOpen(!isOpen)} />
=======
              placeholder="Ask a question..."
              onKeyDown={(e) => e.key === 'Enter' && sendMessage()}
            />
          </div>
        </div>
      )}
      <button className={styles.chatButton} onClick={() => setIsOpen(!isOpen)}>
        ?
      </button>
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
    </div>
  );
};

export default Chatbot;
