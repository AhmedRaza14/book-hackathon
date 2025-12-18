import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './UrduTranslationToggle.module.css';

type UrduTranslationToggleProps = {
  children: React.ReactNode;
};

export default function UrduTranslationToggle({
  children,
}: UrduTranslationToggleProps): JSX.Element {
  const [showUrdu, setShowUrdu] = useState(false);

  // Extract English and Urdu content from children
  const childrenArray = React.Children.toArray(children);
  let englishContent = null;
  let urduContent = null;

  React.Children.forEach(childrenArray, (child: any) => {
    if (child && typeof child === 'object' && child.props) {
      if (child.props.className && child.props.className.includes('english-content')) {
        englishContent = child.props.children;
      } else if (child.props.className && child.props.className.includes('urdu-content')) {
        urduContent = child.props.children;
      }
    }
  });

  const toggleLanguage = () => {
    setShowUrdu(!showUrdu);
  };

  return (
    <div className={styles.translationContainer}>
      <div className={styles.toggleContainer}>
        <button
          className={clsx(
            'button button--sm',
            showUrdu ? 'button--secondary' : 'button--primary',
            styles.toggleButton
          )}
          onClick={toggleLanguage}
        >
          {showUrdu ? 'English' : 'اردو'}
        </button>
        <span className={styles.toggleLabel}>
          {showUrdu ? 'Switch to English' : 'اردو میں تبدیل کریں'}
        </span>
      </div>

      <div className={styles.contentContainer}>
        {showUrdu && urduContent ? (
          <div className={clsx(styles.content, styles.urduContent)} dir="rtl">
            {urduContent}
          </div>
        ) : (
          <div className={clsx(styles.content, styles.englishContent)}>
            {englishContent || children}
          </div>
        )}
      </div>
    </div>
  );
}