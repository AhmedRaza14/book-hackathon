import React, { useState } from 'react';
import clsx from 'clsx';
import styles from './InteractiveCodeBlock.module.css';
import CodeBlock from '@theme/CodeBlock';

type InteractiveCodeBlockProps = {
  language: string;
  framework?: string;
  environment?: string;
  solutionAvailable?: boolean;
  hint?: string;
  children: string;
};

export default function InteractiveCodeBlock({
  language,
  framework,
  environment,
  solutionAvailable = false,
  hint,
  children,
}: InteractiveCodeBlockProps): JSX.Element {
  const [code, setCode] = useState(children.trim());
  const [isSubmitted, setIsSubmitted] = useState(false);
  const [output, setOutput] = useState('');

  const handleSubmit = () => {
    // In a real implementation, this would send the code to a backend for execution
    // For now, we'll just show a placeholder message
    setOutput(`Code execution result would appear here. Framework: ${framework || 'N/A'}, Environment: ${environment || 'N/A'}`);
    setIsSubmitted(true);
  };

  const handleReset = () => {
    setCode(children.trim());
    setOutput('');
    setIsSubmitted(false);
  };

  return (
    <div className={styles.interactiveCodeBlock}>
      <div className={styles.codeHeader}>
        <h4 className={styles.codeTitle}>Interactive Code Block</h4>
        {framework && <span className={styles.frameworkTag}>{framework}</span>}
        {environment && <span className={styles.environmentTag}>{environment}</span>}
      </div>

      <CodeBlock
        language={language}
        onCopy={true}
        onPointerEnter={true}
      >
        {code}
      </CodeBlock>

      {hint && (
        <div className={styles.hint}>
          <strong>Hint:</strong> {hint}
        </div>
      )}

      <div className={styles.controls}>
        <button className={clsx('button button--primary', styles.submitButton)} onClick={handleSubmit}>
          Run Code
        </button>
        <button className={clsx('button button--secondary', styles.resetButton)} onClick={handleReset}>
          Reset
        </button>
        {solutionAvailable && (
          <button className={clsx('button button--info', styles.solutionButton)}>
            Show Solution
          </button>
        )}
      </div>

      {isSubmitted && output && (
        <div className={styles.output}>
          <h5>Output:</h5>
          <pre className={styles.outputContent}>{output}</pre>
        </div>
      )}
    </div>
  );
}