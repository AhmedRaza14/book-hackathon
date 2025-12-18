import React, { useState, useRef, useEffect } from 'react';
import clsx from 'clsx';
import styles from './ChatWidget.module.css';

type Message = {
  id: string;
  text: string;
  sender: 'user' | 'bot';
  timestamp: Date;
};

export default function ChatWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSendMessage = async (e: React.FormEvent) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages((prev) => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Simulate API call to backend
      // In a real implementation, this would call the backend API
      setTimeout(() => {
        const botMessage: Message = {
          id: (Date.now() + 1).toString(),
          text: `I received your message: "${inputValue}". This is a simulated response from the Physical AI textbook assistant. In a real implementation, this would connect to the RAG system to provide context-aware responses based on the textbook content.`,
          sender: 'bot',
          timestamp: new Date(),
        };

        setMessages((prev) => [...prev, botMessage]);
        setIsLoading(false);
      }, 1000);
    } catch (error) {
      console.error('Error sending message:', error);
      setIsLoading(false);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={styles.chatContainer}>
      {isOpen ? (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h4>Physical AI Assistant</h4>
            <button className={styles.closeButton} onClick={toggleChat}>
              ×
            </button>
          </div>
          <div className={styles.chatMessages}>
            {messages.length === 0 ? (
              <div className={styles.welcomeMessage}>
                <p>Hello! I'm your Physical AI & Humanoid Robotics assistant.</p>
                <p>Ask me anything about the textbook content, and I'll provide context-aware responses.</p>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={clsx(
                    styles.message,
                    message.sender === 'user' ? styles.userMessage : styles.botMessage
                  )}
                >
                  <div className={styles.messageContent}>{message.text}</div>
                  <div className={styles.messageTime}>
                    {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                  </div>
                </div>
              ))
            )}
            {isLoading && (
              <div className={clsx(styles.message, styles.botMessage)}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>
          <form onSubmit={handleSendMessage} className={styles.chatInputForm}>
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about Physical AI, ROS 2, Isaac, etc..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button
              type="submit"
              className={clsx('button button--primary', styles.sendButton)}
              disabled={isLoading || !inputValue.trim()}
            >
              Send
            </button>
          </form>
        </div>
      ) : (
        <button className={clsx('button button--primary', styles.floatingButton)} onClick={toggleChat}>
          <span className={styles.chatIcon}>💬</span>
          Ask AI
        </button>
      )}
    </div>
  );
}