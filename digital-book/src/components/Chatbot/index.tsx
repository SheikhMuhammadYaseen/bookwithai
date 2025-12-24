import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

const CHAT_STORAGE_KEY = 'chatbot_history';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [messages, setMessages] = useState<Array<{ role: string; content: string }>>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    const savedChat = localStorage.getItem(CHAT_STORAGE_KEY);
    if (savedChat) {
      try {
        const parsed = JSON.parse(savedChat);
        setMessages(parsed);
      } catch (err) {
        console.error('Failed to load chat history:', err);
      }
    }
  }, []);

  useEffect(() => {
    if (messages.length > 0) {
      localStorage.setItem(CHAT_STORAGE_KEY, JSON.stringify(messages));
    }
  }, [messages]);

  const handleSelection = () => {
    const text = window.getSelection()?.toString().trim();
    if (!text || text === selectedText) return;

    setSelectedText(text);
    setIsOpen(true);

    const userMessage = `Tell me more about: "${text}"`;
    setMessages((prev) => [...prev, { role: 'user', content: userMessage }]);

    sendMessageWithText(userMessage, text);
  };

  const sendMessageWithText = async (userMessage: string, context: string) => {
    setMessages((prev) => [...prev, { role: 'assistant', content: 'thinking' }]);
    setIsLoading(true);

    try {
      const response = await fetch('https://ys5555-aibook-backend.hf.space/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message: userMessage, context }),
      });

      if (!response.ok) throw new Error('API request failed');

      const data = await response.json();

      setMessages((prev) => {
        const newMessages = [...prev];
        newMessages[newMessages.length - 1] = { role: 'assistant', content: data.response };
        return newMessages;
      });
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages((prev) => {
        const newMessages = [...prev];
        newMessages[newMessages.length - 1] = {
          role: 'assistant',
          content: "Sorry, I'm having trouble connecting.",
        };
        return newMessages;
      });
    } finally {
      setIsLoading(false);
    }
  };

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = input.trim();
    setMessages((prev) => [...prev, { role: 'user', content: userMessage }]);
    setInput('');

    setMessages((prev) => [...prev, { role: 'assistant', content: 'thinking' }]);
    setIsLoading(true);

    try {
      const response = await fetch('https://ys5555-aibook-backend.hf.space/api/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ message: userMessage, context: selectedText }),
      });

      if (!response.ok) throw new Error('API request failed');

      const data = await response.json();

      setMessages((prev) => {
        const newMessages = [...prev];
        newMessages[newMessages.length - 1] = { role: 'assistant', content: data.response };
        return newMessages;
      });
    } catch (error) {
      console.error('Error sending message:', error);
      setMessages((prev) => {
        const newMessages = [...prev];
        newMessages[newMessages.length - 1] = {
          role: 'assistant',
          content: "Sorry, I'm having trouble connecting.",
        };
        return newMessages;
      });
    } finally {
      setIsLoading(false);
    }
  };

  useEffect(() => {
    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, [selectedText]);

  const clearChat = () => {
    setMessages([]);
    localStorage.removeItem(CHAT_STORAGE_KEY);
  };

  return (
    <div className={styles.chatContainer}>
      {isOpen && (
        <div className={styles.chatDrawer}>
          <div className={styles.chatHeader}>
            Chat with the Book
            <button className={styles.closeButton} onClick={() => setIsOpen(false)}>
              ×
            </button>
          </div>

          <div className={styles.chatMessages}>
            {messages.length === 0 && (
              <div className={styles.emptyMessage}>Need assistance? I’m here.</div>
            )}
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
          </div>

          <div className={styles.chatInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              placeholder="Ask a question or follow up..."
              onKeyDown={(e) => {
                if (e.key === 'Enter' && !e.shiftKey) {
                  e.preventDefault();
                  sendMessage();
                }
              }}
              rows={3}
            />

            <div className={styles.buttonGroup}>
              <button
                className={styles.clearButton}
                onClick={clearChat}
                disabled={isLoading || messages.length === 0}
              >
                Clear
              </button>
              <button
                onClick={sendMessage}
                disabled={isLoading || !input.trim()}
              >
                Send
              </button>
            </div>
          </div>
        </div>
      )}

      <button className={styles.chatButton} onClick={() => setIsOpen(!isOpen)} />
    </div>
  );
};

export default Chatbot;
