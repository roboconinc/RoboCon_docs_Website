import React, { useEffect, useRef } from 'react';
import styles from './styles.module.css';

interface DiagramDownloadProps {
  diagramId?: string;
  filename?: string;
}

export default function DiagramDownload({ 
  diagramId,
  filename = 'diagram'
}: DiagramDownloadProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const buttonContainerRef = useRef<HTMLDivElement>(null);

  const findDiagram = (): SVGElement | null => {
    // Look for mermaid diagram in parent elements or following siblings
    let current = containerRef.current?.parentElement;
    
    // First check in parent and siblings
    while (current) {
      // Check for mermaid container with SVG
      const mermaidContainer = current.querySelector('.mermaid');
      if (mermaidContainer) {
        const svg = mermaidContainer.querySelector('svg');
        if (svg) {
          return svg as SVGElement;
        }
      }
      
      // Check for mermaid SVG directly
      const mermaidSvg = current.querySelector('.mermaid svg, svg.mermaid');
      if (mermaidSvg) {
        return mermaidSvg as SVGElement;
      }
      
      // Check next sibling for mermaid diagram (in case diagram renders after the button)
      let nextSibling = containerRef.current?.nextElementSibling;
      while (nextSibling) {
        const mermaidInSibling = nextSibling.querySelector('.mermaid svg, svg.mermaid');
        if (mermaidInSibling) {
          return mermaidInSibling as SVGElement;
        }
        nextSibling = nextSibling.nextElementSibling;
      }
      
      current = current.parentElement;
    }
    return null;
  };

  useEffect(() => {
    // Wait for diagrams to render, then position buttons
    const checkAndPosition = () => {
      if (!containerRef.current) return;
      
      const diagram = findDiagram();
      if (!diagram) {
        // Hide buttons if no diagram found after a delay
        setTimeout(() => {
          const diagramRetry = findDiagram();
          if (!diagramRetry && buttonContainerRef.current) {
            buttonContainerRef.current.style.display = 'none';
          }
        }, 1000);
        return;
      }

      // Try to move buttons to appear after the diagram
      const diagramParent = diagram.closest('.mermaid') || diagram.parentElement;
      if (diagramParent && containerRef.current && diagramParent.nextSibling !== containerRef.current) {
        // Check if we're already positioned correctly
        const currentParent = containerRef.current.parentElement;
        if (currentParent !== diagramParent.parentElement) {
          // Move buttons to appear after the diagram container
          diagramParent.parentElement?.insertBefore(containerRef.current, diagramParent.nextSibling);
        }
      }
    };

    // Check immediately and after a delay (for async rendering)
    checkAndPosition();
    const timeout = setTimeout(checkAndPosition, 500);
    const interval = setInterval(checkAndPosition, 1000);
    
    // Clean up after 5 seconds
    setTimeout(() => {
      clearInterval(interval);
    }, 5000);

    return () => {
      clearTimeout(timeout);
      clearInterval(interval);
    };
  }, []);

  const handleDownloadSVG = () => {
    try {
      const diagram = findDiagram();
      if (!diagram) {
        alert('Could not find diagram to download');
        return;
      }

      // Clone the SVG to avoid modifying the original
      const clonedSvg = diagram.cloneNode(true) as SVGElement;
      
      // Get the bounding box and set explicit dimensions
      const bbox = diagram.getBBox();
      const width = bbox.width || diagram.clientWidth || 800;
      const height = bbox.height || diagram.clientHeight || 600;

      clonedSvg.setAttribute('width', width.toString());
      clonedSvg.setAttribute('height', height.toString());
      clonedSvg.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
      clonedSvg.setAttribute('xmlns:xlink', 'http://www.w3.org/1999/xlink');
      
      // Preserve viewBox if it exists, otherwise set it
      if (!clonedSvg.getAttribute('viewBox')) {
        clonedSvg.setAttribute('viewBox', `${bbox.x} ${bbox.y} ${width} ${height}`);
      }

      // Serialize SVG to string
      const svgData = new XMLSerializer().serializeToString(clonedSvg);
      
      // Create SVG blob with proper XML declaration
      const svgBlob = new Blob([`<?xml version="1.0" encoding="UTF-8"?>\n${svgData}`], { 
        type: 'image/svg+xml;charset=utf-8' 
      });

      // Create download link
      const url = URL.createObjectURL(svgBlob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `${filename}.svg`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);
    } catch (error) {
      console.error('Error downloading SVG:', error);
      alert('Error generating SVG. Please try again.');
    }
  };

  const handleDownloadPNG = async () => {
    try {
      const diagram = findDiagram();
      if (!diagram) {
        alert('Could not find diagram to download');
        return;
      }

      // Clone the SVG to avoid modifying the original
      const clonedSvg = diagram.cloneNode(true) as SVGElement;
      
      // Get the bounding box
      const bbox = diagram.getBBox();
      const width = bbox.width || diagram.clientWidth || 800;
      const height = bbox.height || diagram.clientHeight || 600;

      // Set explicit width and height on cloned SVG
      clonedSvg.setAttribute('width', width.toString());
      clonedSvg.setAttribute('height', height.toString());
      clonedSvg.setAttribute('viewBox', `${bbox.x} ${bbox.y} ${width} ${height}`);

      // Serialize SVG to string
      const svgData = new XMLSerializer().serializeToString(clonedSvg);
      
      // Create an image from the SVG
      const svgBlob = new Blob([svgData], { type: 'image/svg+xml;charset=utf-8' });
      const svgUrl = URL.createObjectURL(svgBlob);

      const img = new Image();
      
      img.onload = () => {
        // Create canvas and draw image
        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        
        if (!ctx) {
          alert('Could not create canvas context');
          URL.revokeObjectURL(svgUrl);
          return;
        }

        // Set canvas size (with padding for better quality)
        const scale = 2; // Higher resolution
        canvas.width = width * scale;
        canvas.height = height * scale;
        
        // Fill white background
        ctx.fillStyle = '#ffffff';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        
        // Draw the image
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        
        // Convert to PNG
        canvas.toBlob((blob) => {
          if (!blob) {
            alert('Could not generate PNG');
            URL.revokeObjectURL(svgUrl);
            return;
          }

          // Create download link
          const url = URL.createObjectURL(blob);
          const link = document.createElement('a');
          link.href = url;
          link.download = `${filename}.png`;
          document.body.appendChild(link);
          link.click();
          document.body.removeChild(link);
          URL.revokeObjectURL(url);
          URL.revokeObjectURL(svgUrl);
        }, 'image/png');
      };

      img.onerror = () => {
        alert('Error loading diagram image');
        URL.revokeObjectURL(svgUrl);
      };

      img.src = svgUrl;
    } catch (error) {
      console.error('Error downloading diagram:', error);
      alert('Error generating PNG. Please try again.');
    }
  };

  return (
    <div ref={containerRef} className={styles.diagramDownload}>
      <div ref={buttonContainerRef} className={styles.buttonContainer}>
        <button
          className={styles.downloadButton}
          onClick={handleDownloadSVG}
          aria-label="Download diagram as SVG"
          title="Download diagram as SVG"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
            <polyline points="7 10 12 15 17 10" />
            <line x1="12" y1="15" x2="12" y2="3" />
          </svg>
          Download SVG
        </button>
        <button
          className={styles.downloadButton}
          onClick={handleDownloadPNG}
          aria-label="Download diagram as PNG"
          title="Download diagram as PNG"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
            <polyline points="7 10 12 15 17 10" />
            <line x1="12" y1="15" x2="12" y2="3" />
          </svg>
          Download PNG
        </button>
      </div>
    </div>
  );
}

