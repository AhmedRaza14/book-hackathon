import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './PersonalizeButton.module.css';

type PersonalizeButtonProps = {
  userId?: string;
  defaultLevel?: 'beginner' | 'intermediate' | 'advanced';
};

export default function PersonalizeButton({
  userId,
  defaultLevel = 'intermediate',
}: PersonalizeButtonProps): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [level, setLevel] = useState(defaultLevel);
  const [hardwareFilter, setHardwareFilter] = useState(true);

  const toggleMenu = () => {
    setIsOpen(!isOpen);
  };

  const handleLevelChange = (newLevel: 'beginner' | 'intermediate' | 'advanced') => {
    setLevel(newLevel);
    setIsOpen(false);
    // In a real implementation, this would update user preferences in the backend
    console.log(`User level set to: ${newLevel}`);
  };

  const handleHardwareFilterChange = () => {
    setHardwareFilter(!hardwareFilter);
    // In a real implementation, this would update user preferences in the backend
    console.log(`Hardware filter set to: ${!hardwareFilter}`);
  };

  return (
    <div className={styles.personalizeContainer}>
      <button
        className={clsx('button button--primary', styles.personalizeButton)}
        onClick={toggleMenu}
        aria-expanded={isOpen}
      >
        <span className={styles.buttonIcon}>⚙️</span>
        Personalize
      </button>

      {isOpen && (
        <div className={styles.dropdownMenu}>
          <div className={styles.menuHeader}>
            <h4>Personalize Content</h4>
            <button
              className={styles.closeButton}
              onClick={toggleMenu}
              aria-label="Close menu"
            >
              ×
            </button>
          </div>

          <div className={styles.menuContent}>
            <div className={styles.optionGroup}>
              <h5>Experience Level</h5>
              <div className={styles.levelOptions}>
                <button
                  className={clsx(
                    'button button--sm',
                    level === 'beginner' ? 'button--primary' : 'button--outline'
                  )}
                  onClick={() => handleLevelChange('beginner')}
                >
                  Beginner
                </button>
                <button
                  className={clsx(
                    'button button--sm',
                    level === 'intermediate' ? 'button--primary' : 'button--outline'
                  )}
                  onClick={() => handleLevelChange('intermediate')}
                >
                  Intermediate
                </button>
                <button
                  className={clsx(
                    'button button--sm',
                    level === 'advanced' ? 'button--primary' : 'button--outline'
                  )}
                  onClick={() => handleLevelChange('advanced')}
                >
                  Advanced
                </button>
              </div>
            </div>

            <div className={styles.optionGroup}>
              <label className={styles.checkboxLabel}>
                <input
                  type="checkbox"
                  checked={hardwareFilter}
                  onChange={handleHardwareFilterChange}
                  className={styles.checkbox}
                />
                <span>Show content based on my hardware</span>
              </label>
            </div>

            <div className={styles.optionGroup}>
              <h5>Current Selection</h5>
              <p>
                <strong>Level:</strong> {level.charAt(0).toUpperCase() + level.slice(1)}<br />
                <strong>Hardware Filter:</strong> {hardwareFilter ? 'Enabled' : 'Disabled'}
              </p>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}