import React from 'react';
import styles from './styles.module.css';

const InteractiveButtons = () => {
  const handleClick = (feature) => {
    alert(`This ${feature} feature is coming soon!`);
  };

  return (
    <div className={styles.buttonContainer}>
      <button className={styles.interactiveButton} onClick={() => handleClick('theory exercise')}>Theory Exercise</button>
      <button className={styles.interactiveButton} onClick={() => handleClick('coding exercise')}>Coding Exercise</button>
      <button className={styles.interactiveButton} onClick={() => handleClick('quiz')}>Quiz</button>
      <button className={styles.interactiveButton} onClick={() => handleClick('flashcard')}>Flashcard</button>
      <button className={styles.interactiveButton} onClick={() => handleClick('summary')}>Summary</button>
      <button className={styles.interactiveButton} onClick={() => handleClick('video')}>Video</button>
    </div>
  );
};

export default InteractiveButtons;
