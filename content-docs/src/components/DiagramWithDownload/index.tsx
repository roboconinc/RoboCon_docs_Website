import React from 'react';
import DiagramDownload from '@site/src/components/DiagramDownload';
import styles from './styles.module.css';

interface DiagramWithDownloadProps {
  filename: string;
  children: React.ReactNode;
}

export default function DiagramWithDownload({ filename, children }: DiagramWithDownloadProps) {
  return (
    <div className={styles.diagramWrapper}>
      <div className={styles.diagramContent}>
        {children}
      </div>
      <div className={styles.downloadContainer}>
        <DiagramDownload filename={filename} />
      </div>
    </div>
  );
}

