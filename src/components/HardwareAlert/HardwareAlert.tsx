import React from 'react';
import clsx from 'clsx';
import styles from './HardwareAlert.module.css';

type HardwareAlertType = 'caution' | 'danger' | 'info';
type HardwareAlertProps = {
  requirement: string;
  minimum: string;
  recommended: string;
  purpose: string;
  type?: HardwareAlertType;
};

const HardwareAlertTypes = {
  info: {
    icon: '💡',
    color: 'info',
  },
  caution: {
    icon: '⚠️',
    color: 'warning',
  },
  danger: {
    icon: '❌',
    color: 'danger',
  },
};

export default function HardwareAlert({
  requirement,
  minimum,
  recommended,
  purpose,
  type = 'info',
}: HardwareAlertProps): JSX.Element {
  const hardwareAlert = HardwareAlertTypes[type];
  const icon = hardwareAlert.icon;
  const color = hardwareAlert.color;

  return (
    <div className={clsx('alert alert--' + color, styles.hardwareAlert)}>
      <div className={styles.hardwareAlertHeader}>
        <span className={styles.hardwareAlertIcon}>{icon}</span>
        <h5 className={styles.hardwareAlertTitle}>Hardware Requirement</h5>
      </div>
      <div className={styles.hardwareAlertContent}>
        <div className={styles.hardwareAlertItem}>
          <strong>Requirement:</strong> {requirement}
        </div>
        <div className={styles.hardwareAlertItem}>
          <strong>Minimum:</strong> {minimum}
        </div>
        <div className={styles.hardwareAlertItem}>
          <strong>Recommended:</strong> {recommended}
        </div>
        <div className={styles.hardwareAlertItem}>
          <strong>Purpose:</strong> {purpose}
        </div>
      </div>
    </div>
  );
}