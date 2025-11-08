import React from 'react';
import styles from './styles.module.css';
import { asBlob } from 'html-docx-js-typescript';

interface DownloadButtonsProps {
  title?: string;
  filename?: string;
}

export default function DownloadButtons({ 
  title = 'Document',
  filename = 'document'
}: DownloadButtonsProps) {
  
  const handleDownloadPDF = async () => {
    // Show loading indicator
    const originalText = 'Download PDF';
    const button = document.querySelector('[aria-label="Download as PDF"]') as HTMLButtonElement;
    if (button) {
      button.disabled = true;
      button.textContent = 'Processing...';
    }

    try {
      // Get the main content area
      const content = document.querySelector('article') || document.querySelector('main') || document.body;
      
      if (!content) {
        alert('Could not find content to export');
        if (button) {
          button.disabled = false;
          button.textContent = originalText;
        }
        return;
      }

      // Clone the content to avoid modifying the original
      const clonedContent = content.cloneNode(true) as HTMLElement;
      
      // Remove unwanted elements (including download buttons)
      const elementsToRemove = clonedContent.querySelectorAll('script, style, .downloadButtons, nav, aside, footer, header, .navbar, .diagramDownload, button[aria-label*="Download"]');
      elementsToRemove.forEach(el => el.remove());

      // Convert Mermaid diagrams to images for PDF
      const convertMermaidForPDF = async () => {
        // First, find the original Mermaid diagrams in the document (not cloned)
        const originalMermaidContainers = document.querySelectorAll('.mermaid, pre.mermaid');
        
        const mermaidPromises = Array.from(originalMermaidContainers).map(async (originalContainer, index) => {
          try {
            // Find the corresponding container in cloned content
            const clonedContainers = clonedContent.querySelectorAll('.mermaid, pre.mermaid');
            const container = clonedContainers[index] as HTMLElement;
            if (!container) {
              return;
            }

            // Find the SVG inside the original mermaid container (from the actual DOM)
            const originalSvg = originalContainer.querySelector('svg') as SVGElement;
            if (!originalSvg) {
              return;
            }

            // Get SVG dimensions from the original
            let width = originalSvg.clientWidth || originalSvg.getBoundingClientRect().width || 800;
            let height = originalSvg.clientHeight || originalSvg.getBoundingClientRect().height || 600;
            
            // If dimensions are 0, try getBBox
            if (width === 0 || height === 0) {
              try {
                const bbox = originalSvg.getBBox();
                width = bbox.width || 800;
                height = bbox.height || 600;
              } catch (e) {
                width = 800;
                height = 600;
              }
            }

            // Clone SVG from original (not cloned content) to get actual rendered version
            const clonedSvg = originalSvg.cloneNode(true) as SVGElement;
            
            // Ensure SVG has proper attributes
            clonedSvg.setAttribute('width', width.toString());
            clonedSvg.setAttribute('height', height.toString());
            if (!clonedSvg.getAttribute('xmlns')) {
              clonedSvg.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
            }
            if (!clonedSvg.getAttribute('xmlns:xlink')) {
              clonedSvg.setAttribute('xmlns:xlink', 'http://www.w3.org/1999/xlink');
            }

            // Force light theme colors - fix dark theme issues for PDF
            // Use DOM manipulation first for more reliable text color fixing
            const textElements = clonedSvg.querySelectorAll('text, tspan');
            textElements.forEach((textEl) => {
              const textElement = textEl as SVGTextElement;
              // Force black fill directly on the element - use setAttribute for highest priority
              textElement.setAttribute('fill', '#000000');
              textElement.setAttributeNS(null, 'fill', '#000000');
              
              // Remove any style that might override
              const style = textElement.getAttribute('style') || '';
              let newStyle = style
                .replace(/fill:\s*[^;]+;?/gi, '')
                .replace(/fill:\s*[^;]+/gi, '')
                .trim();
              // Clean up
              newStyle = newStyle.replace(/;;+/g, ';').replace(/^;|;$/g, '');
              if (newStyle) {
                textElement.setAttribute('style', newStyle);
              } else {
                textElement.removeAttribute('style');
              }
              
              // Remove class that might affect color
              textElement.removeAttribute('class');
              
              // Also check parent elements for fill that might be inherited
              let parent = textElement.parentElement;
              let depth = 0;
              while (parent && parent !== clonedSvg && depth < 5) {
                const parentFill = parent.getAttribute('fill');
                if (parentFill && (parentFill === '#1e1e1e' || parentFill === '#2d2d2d' || parentFill === '#212121' || parentFill === '#000000')) {
                  // Don't inherit dark fills - text should be black
                  textElement.setAttribute('fill', '#000000');
                }
                parent = parent.parentElement;
                depth++;
              }
            });
            
            // Also process any style tags that might contain text color rules
            const styleTags = clonedSvg.querySelectorAll('style');
            styleTags.forEach((styleTag) => {
              let styleContent = styleTag.textContent || '';
              // Remove any fill rules that might affect text
              styleContent = styleContent
                .replace(/text\s*\{[^}]*fill:[^}]*\}/gi, 'text { fill: #000000 !important; }')
                .replace(/tspan\s*\{[^}]*fill:[^}]*\}/gi, 'tspan { fill: #000000 !important; }')
                .replace(/fill:\s*[^;]+;?/gi, (match) => {
                  // If it's in a text or tspan rule, force black
                  return 'fill: #000000 !important;';
                });
              styleTag.textContent = styleContent;
            });
            
            // Now serialize to string
            const svgString = new XMLSerializer().serializeToString(clonedSvg);
            
            // More comprehensive color replacement for PDF rendering
            // Strategy: Make all backgrounds white, all text black, all strokes black
            let processedSvg = svgString;
            
            // STEP 1: Force ALL text and tspan elements to have explicit black fill
            // Do this multiple times to catch nested structures
            for (let i = 0; i < 3; i++) {
              processedSvg = processedSvg.replace(/<text([^>]*)>/gi, (match, attrs) => {
                // Completely remove fill, style fill, and class
                let newAttrs = attrs
                  .replace(/fill="[^"]*"/gi, '')
                  .replace(/style="[^"]*"/gi, (styleMatch) => {
                    return styleMatch.replace(/fill:\s*[^;]+;?/gi, '').replace(/fill:\s*[^;]+/gi, '');
                  })
                  .replace(/class="[^"]*"/gi, '');
                
                // Always add explicit black fill - make it first attribute for priority
                newAttrs = ' fill="#000000"' + newAttrs;
                
                return `<text${newAttrs}>`;
              });
              
              processedSvg = processedSvg.replace(/<tspan([^>]*)>/gi, (match, attrs) => {
                let newAttrs = attrs
                  .replace(/fill="[^"]*"/gi, '')
                  .replace(/style="[^"]*"/gi, (styleMatch) => {
                    return styleMatch.replace(/fill:\s*[^;]+;?/gi, '').replace(/fill:\s*[^;]+/gi, '');
                  });
                
                newAttrs = ' fill="#000000"' + newAttrs;
                
                return `<tspan${newAttrs}>`;
              });
            }
            
            // STEP 2: Process nested groups - ensure text in groups is black
            processedSvg = processedSvg.replace(/<g([^>]*)>([\s\S]*?)<\/g>/gi, (match, attrs, content) => {
              if (content.includes('<text') || content.includes('<tspan')) {
                // Process all text elements in this group
                content = content.replace(/<text([^>]*)>/gi, (textMatch, textAttrs) => {
                  let newTextAttrs = textAttrs.replace(/fill="[^"]*"/gi, '').replace(/style="[^"]*fill:[^"]*"/gi, '');
                  return `<text fill="#000000"${newTextAttrs}>`;
                });
                content = content.replace(/<tspan([^>]*)>/gi, (tspanMatch, tspanAttrs) => {
                  let newTspanAttrs = tspanAttrs.replace(/fill="[^"]*"/gi, '').replace(/style="[^"]*fill:[^"]*"/gi, '');
                  return `<tspan fill="#000000"${newTspanAttrs}>`;
                });
                return `<g${attrs}>${content}</g>`;
              }
              return match;
            });
            
            // STEP 3: Replace dark background colors with white (for shapes only)
            processedSvg = processedSvg
              .replace(/fill="#1e1e1e"/gi, 'fill="#ffffff"')
              .replace(/fill="#2d2d2d"/gi, 'fill="#ffffff"')
              .replace(/fill="#212121"/gi, 'fill="#ffffff"')
              .replace(/fill="rgb\(30,\s*30,\s*30\)"/gi, 'fill="#ffffff"')
              .replace(/fill="rgb\(45,\s*45,\s*45\)"/gi, 'fill="#ffffff"')
              .replace(/fill="rgb\(33,\s*33,\s*33\)"/gi, 'fill="#ffffff"');
            
            // STEP 4: Handle any remaining black fills - keep for text, white for shapes
            processedSvg = processedSvg.replace(/fill="#000000"/gi, (match, offset, str) => {
              const before = str.substring(Math.max(0, offset - 1000), offset);
              const after = str.substring(offset, Math.min(str.length, offset + 200));
              
              // If near text/tspan tags, keep black
              if (before.match(/<text[^>]*>/) || before.match(/<tspan[^>]*>/) || 
                  after.match(/<text[^>]*>/) || after.match(/<tspan[^>]*>/)) {
                return 'fill="#000000"';
              }
              
              // If in a shape element, make white
              const shapeMatch = before.match(/<(rect|circle|ellipse|path|polygon|polyline|line|g)[^>]*>/);
              if (shapeMatch && !before.match(/<text/) && !before.match(/<tspan/)) {
                return 'fill="#ffffff"';
              }
              
              return 'fill="#000000"'; // Default: keep black (safer)
            });
            
            // STEP 5: Fix white fills that are in text contexts
            processedSvg = processedSvg.replace(/fill="#ffffff"/g, (match, offset, str) => {
              const before = str.substring(Math.max(0, offset - 1000), offset);
              const after = str.substring(offset, Math.min(str.length, offset + 200));
              
              if (before.match(/<text[^>]*>/) || before.match(/<tspan[^>]*>/) || 
                  after.match(/<text[^>]*>/) || after.match(/<tspan[^>]*>/)) {
                return 'fill="#000000"';
              }
              
              return match;
            });
            
            // STEP 6: Final aggressive pass - find ALL text and force black
            // Use a more comprehensive regex to catch all variations
            processedSvg = processedSvg.replace(/(<text[^>]*?)(fill="[^"]*")?([^>]*>)/gi, (match, start, fillAttr, rest) => {
              // Remove any existing fill
              const cleaned = start.replace(/fill="[^"]*"/gi, '');
              return `${cleaned} fill="#000000"${rest}`;
            });
            
            processedSvg = processedSvg.replace(/(<tspan[^>]*?)(fill="[^"]*")?([^>]*>)/gi, (match, start, fillAttr, rest) => {
              const cleaned = start.replace(/fill="[^"]*"/gi, '');
              return `${cleaned} fill="#000000"${rest}`;
            });
            
            // STEP 7: Remove style attributes that might override fill
            processedSvg = processedSvg.replace(/style="([^"]*)"/gi, (match, styleContent) => {
              // Remove fill from styles, but keep other styles
              let newStyle = styleContent
                .replace(/fill:\s*[^;]+;?/gi, '')
                .replace(/fill:\s*[^;]+/gi, '')
                .trim();
              // Clean up double semicolons
              newStyle = newStyle.replace(/;;+/g, ';').replace(/^;|;$/g, '');
              return newStyle ? `style="${newStyle}"` : '';
            });
            
            // STEP 8: Stroke colors - make black for visibility
            processedSvg = processedSvg
              .replace(/stroke="#1e1e1e"/gi, 'stroke="#000000"')
              .replace(/stroke="#2d2d2d"/gi, 'stroke="#000000"')
              .replace(/stroke="#212121"/gi, 'stroke="#000000"')
              .replace(/stroke="rgb\(30,\s*30,\s*30\)"/gi, 'stroke="#000000"')
              .replace(/stroke="rgb\(45,\s*45,\s*45\)"/gi, 'stroke="#000000"')
              .replace(/stroke="none"/gi, 'stroke="transparent"');
            
            // STEP 9: Background colors
            processedSvg = processedSvg
              .replace(/background-color:#1e1e1e/gi, 'background-color:#ffffff')
              .replace(/background-color:#2d2d2d/gi, 'background-color:#ffffff')
              .replace(/background-color:#212121/gi, 'background-color:#ffffff');
            
            // STEP 10: Ensure SVG root has white background
            // Add a white background rect if the SVG doesn't have one
            if (!processedSvg.includes('<rect') || !processedSvg.match(/<rect[^>]*fill="#ffffff"/i)) {
              // Find the opening <svg> tag and add a white background rect right after it
              processedSvg = processedSvg.replace(
                /(<svg[^>]*>)/i,
                `$1<rect width="100%" height="100%" fill="#ffffff"/>`
              );
            }
            
            // STEP 11: Final verification - ensure ALL text elements have black fill
            // One more aggressive pass to catch anything we missed
            processedSvg = processedSvg.replace(/<text[^>]*>/gi, (match) => {
              // Remove all fill attributes and add black fill
              return match.replace(/fill="[^"]*"/gi, '').replace(/>/, ' fill="#000000">');
            });
            processedSvg = processedSvg.replace(/<tspan[^>]*>/gi, (match) => {
              return match.replace(/fill="[^"]*"/gi, '').replace(/>/, ' fill="#000000">');
            });
            
            // STEP 12: Add explicit CSS style block to force all text to be black
            // This is critical for canvas rendering - ensures text is visible
            if (!processedSvg.includes('<style')) {
              processedSvg = processedSvg.replace(
                /(<svg[^>]*>)/i,
                `$1<style type="text/css"><![CDATA[
                  * { color: #000000 !important; }
                  text, tspan { fill: #000000 !important; color: #000000 !important; }
                  text *, tspan * { fill: #000000 !important; color: #000000 !important; }
                ]]></style>`
              );
            } else {
              // Update existing style block to force text colors
              processedSvg = processedSvg.replace(
                /<style[^>]*>([\s\S]*?)<\/style>/i,
                (styleMatch, styleContent) => {
                  return `<style type="text/css"><![CDATA[
                    ${styleContent}
                    * { color: #000000 !important; }
                    text, tspan { fill: #000000 !important; color: #000000 !important; }
                    text *, tspan * { fill: #000000 !important; color: #000000 !important; }
                  ]]></style>`;
                }
              );
            }
            
            // STEP 13: Ensure SVG has white background and is isolated from page styles
            // Add a white background rect as the first element
            if (!processedSvg.match(/<rect[^>]*fill="#ffffff"/i)) {
              processedSvg = processedSvg.replace(
                /(<svg[^>]*>)/i,
                `$1<rect width="100%" height="100%" fill="#ffffff" x="0" y="0"/>`
              );
            }
            
            // STEP 14: Remove any external style references and ensure inline styles
            processedSvg = processedSvg.replace(/xmlns:xlink="[^"]*"/gi, '');
            processedSvg = processedSvg.replace(/<use[^>]*>/gi, ''); // Remove use elements that might reference external styles

            const svgBlob = new Blob([processedSvg], { type: 'image/svg+xml;charset=utf-8' });
            const svgUrl = URL.createObjectURL(svgBlob);

            return new Promise<void>((resolve) => {
              const img = new Image();
              
              // Set up image loading with proper handling
              img.onload = () => {
                try {
                  // Wait longer to ensure image is fully rendered with all text visible
                  setTimeout(() => {
                    try {
                      // Create a temporary container to render the SVG and verify it
                      // Isolate from page styles to prevent dark theme interference
                      const tempContainer = document.createElement('div');
                      tempContainer.style.position = 'absolute';
                      tempContainer.style.left = '-9999px';
                      tempContainer.style.width = width + 'px';
                      tempContainer.style.height = height + 'px';
                      tempContainer.style.backgroundColor = '#ffffff';
                      tempContainer.style.color = '#000000';
                      // Create an isolated container to prevent style inheritance
                      const isolatedContainer = document.createElement('div');
                      isolatedContainer.style.all = 'initial';
                      isolatedContainer.style.display = 'block';
                      isolatedContainer.style.width = width + 'px';
                      isolatedContainer.style.height = height + 'px';
                      isolatedContainer.style.backgroundColor = '#ffffff';
                      isolatedContainer.innerHTML = processedSvg;
                      tempContainer.appendChild(isolatedContainer);
                      document.body.appendChild(tempContainer);
                      
                      // Wait for the SVG to render in the DOM and verify text is visible
                      setTimeout(() => {
                        try {
                          // Verify text elements in the rendered SVG are visible
                          const renderedSvg = isolatedContainer.querySelector('svg') || tempContainer.querySelector('svg');
                          if (renderedSvg) {
                            // First, ensure SVG has white background
                            const existingBg = renderedSvg.querySelector('rect[fill="#ffffff"]');
                            if (!existingBg) {
                              const bgRect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
                              bgRect.setAttribute('width', '100%');
                              bgRect.setAttribute('height', '100%');
                              bgRect.setAttribute('fill', '#ffffff');
                              bgRect.setAttribute('x', '0');
                              bgRect.setAttribute('y', '0');
                              renderedSvg.insertBefore(bgRect, renderedSvg.firstChild);
                            }
                            // CRITICAL FIX: Convert all text elements to paths to ensure visibility
                            // This prevents text color issues by rendering text as shapes
                            const textElements = Array.from(renderedSvg.querySelectorAll('text'));
                            
                            textElements.forEach((textEl) => {
                              try {
                                const textElement = textEl as SVGTextElement;
                                const textContent = textElement.textContent || '';
                                
                                if (!textContent.trim()) return;
                                
                                // Get text properties
                                const x = parseFloat(textElement.getAttribute('x') || '0');
                                const y = parseFloat(textElement.getAttribute('y') || '0');
                                const fontSize = parseFloat(textElement.getAttribute('font-size') || 
                                  window.getComputedStyle(textElement).fontSize || '12');
                                const fontFamily = textElement.getAttribute('font-family') || 
                                  window.getComputedStyle(textElement).fontFamily || 'Arial';
                                const fontWeight = textElement.getAttribute('font-weight') || 
                                  window.getComputedStyle(textElement).fontWeight || 'normal';
                                
                                // Create a temporary canvas to measure and render text as path
                                const tempCanvas = document.createElement('canvas');
                                const tempCtx = tempCanvas.getContext('2d');
                                if (!tempCtx) return;
                                
                                tempCtx.font = `${fontWeight} ${fontSize}px ${fontFamily}`;
                                const metrics = tempCtx.measureText(textContent);
                                
                                // Create a path element to replace the text
                                // For now, let's try a different approach - ensure text is visible
                                // by setting explicit black fill and removing all style inheritance
                                textElement.setAttribute('fill', '#000000');
                                textElement.setAttribute('stroke', 'none');
                                textElement.style.fill = '#000000';
                                textElement.style.color = '#000000';
                                textElement.style.stroke = 'none';
                                textElement.removeAttribute('class');
                                
                                // Remove any parent fill that might be inherited
                                let parent = textElement.parentElement;
                                while (parent && parent !== renderedSvg) {
                                  const parentFill = parent.getAttribute('fill');
                                  if (parentFill && parentFill !== 'none' && parentFill !== 'transparent') {
                                    // Don't inherit - text should be black
                                    textElement.setAttribute('fill', '#000000');
                                  }
                                  parent = parent.parentElement;
                                }
                              } catch (e) {
                                console.warn('Error processing text element:', e);
                              }
                            });
                            
                            // Also process tspan elements
                            const tspanElements = Array.from(renderedSvg.querySelectorAll('tspan'));
                            tspanElements.forEach((tspanEl) => {
                              const tspanElement = tspanEl as SVGTSpanElement;
                              tspanElement.setAttribute('fill', '#000000');
                              tspanElement.style.fill = '#000000';
                              tspanElement.style.color = '#000000';
                              tspanElement.removeAttribute('class');
                            });
                            
                            // Force all groups to not inherit dark fills for text
                            const groups = renderedSvg.querySelectorAll('g');
                            groups.forEach((group) => {
                              const groupFill = group.getAttribute('fill');
                              if (groupFill && (groupFill === '#1e1e1e' || groupFill === '#2d2d2d' || groupFill === '#212121')) {
                                // Remove dark fill from group - text children should be black
                                group.removeAttribute('fill');
                              }
                            });
                            
                            // Verify text is actually visible by checking computed styles
                            const verifyTextVisibility = () => {
                              const allText = renderedSvg.querySelectorAll('text, tspan');
                              let allTextVisible = true;
                              
                              allText.forEach((textEl) => {
                                const computedStyle = window.getComputedStyle(textEl);
                                const fill = computedStyle.fill || textEl.getAttribute('fill');
                                const color = computedStyle.color;
                                
                                // Check if fill is dark (should be black #000000)
                                if (fill && fill !== 'rgb(0, 0, 0)' && fill !== '#000000' && fill !== 'black' && fill !== 'none' && fill !== 'transparent') {
                                  // Force to black
                                  (textEl as SVGElement).setAttribute('fill', '#000000');
                                  (textEl as SVGElement).style.fill = '#000000';
                                  allTextVisible = false;
                                }
                                
                                // Also check if it's inheriting a dark color
                                if (color && color !== 'rgb(0, 0, 0)' && color !== '#000000' && color !== 'black') {
                                  (textEl as SVGElement).style.color = '#000000';
                                  allTextVisible = false;
                                }
                              });
                              
                              return allTextVisible;
                            };
                            
                            // Verify and fix text visibility
                            let attempts = 0;
                            while (attempts < 3 && !verifyTextVisibility()) {
                              // Force all text to black again
                              renderedSvg.querySelectorAll('text, tspan').forEach((textEl) => {
                                (textEl as SVGElement).setAttribute('fill', '#000000');
                                (textEl as SVGElement).style.fill = '#000000';
                                (textEl as SVGElement).style.color = '#000000';
                              });
                              attempts++;
                            }
                            
                            // Re-serialize the SVG with the fixed text
                            const fixedSvgString = new XMLSerializer().serializeToString(renderedSvg);
                            
                            // Aggressive string processing to ensure ALL text is black
                            let finalSvgString = fixedSvgString
                              // Remove all fill attributes from text and add black
                              .replace(/<text([^>]*)>/gi, (match, attrs) => {
                                let newAttrs = attrs
                                  .replace(/fill="[^"]*"/gi, '')
                                  .replace(/style="[^"]*"/gi, '')
                                  .replace(/class="[^"]*"/gi, '');
                                return `<text${newAttrs} fill="#000000" style="fill: #000000 !important; stroke: none;">`;
                              })
                              .replace(/<tspan([^>]*)>/gi, (match, attrs) => {
                                let newAttrs = attrs
                                  .replace(/fill="[^"]*"/gi, '')
                                  .replace(/style="[^"]*"/gi, '')
                                  .replace(/class="[^"]*"/gi, '');
                                return `<tspan${newAttrs} fill="#000000" style="fill: #000000 !important; stroke: none;">`;
                              })
                              // Remove dark fills from groups that contain text
                              .replace(/<g([^>]*)fill="(1e1e1e|2d2d2d|212121)"([^>]*)>/gi, (match, before, fillColor, after) => {
                                return `<g${before}${after}>`;
                              });
                            
                            // Add comprehensive style block
                            if (!finalSvgString.includes('<style')) {
                              finalSvgString = finalSvgString.replace(
                                /(<svg[^>]*>)/i,
                                `$1<style type="text/css"><![CDATA[
                                  * { color: #000000 !important; }
                                  text, tspan { fill: #000000 !important; stroke: none !important; }
                                  text *, tspan * { fill: #000000 !important; stroke: none !important; }
                                  g[fill="#1e1e1e"] text, g[fill="#2d2d2d"] text, g[fill="#212121"] text { fill: #000000 !important; }
                                ]]></style>`
                              );
                            } else {
                              // Update existing style
                              finalSvgString = finalSvgString.replace(
                                /<style[^>]*>([\s\S]*?)<\/style>/i,
                                `<style type="text/css"><![CDATA[
                                  * { color: #000000 !important; }
                                  text, tspan { fill: #000000 !important; stroke: none !important; }
                                  text *, tspan * { fill: #000000 !important; stroke: none !important; }
                                  g[fill="#1e1e1e"] text, g[fill="#2d2d2d"] text, g[fill="#212121"] text { fill: #000000 !important; }
                                ]]></style>`
                              );
                            }
                            
                            const fixedSvgBlob = new Blob([finalSvgString], { type: 'image/svg+xml;charset=utf-8' });
                            const fixedSvgUrl = URL.createObjectURL(fixedSvgBlob);
                            
                            // Create new image with fixed SVG
                            const fixedImg = new Image();
                            fixedImg.crossOrigin = 'anonymous';
                            
                            fixedImg.onload = () => {
                              // Wait longer to ensure image is fully loaded and rendered
                              setTimeout(() => {
                                try {
                                  // First, verify the image actually loaded with content
                                  if (fixedImg.naturalWidth === 0 || fixedImg.naturalHeight === 0) {
                                    console.warn('SVG image has zero dimensions');
                                    document.body.removeChild(tempContainer);
                                    URL.revokeObjectURL(svgUrl);
                                    URL.revokeObjectURL(fixedSvgUrl);
                                    resolve();
                                    return;
                                  }
                                  
                                  const canvas = document.createElement('canvas');
                                  // Use actual image dimensions
                                  const canvasWidth = Math.max(fixedImg.naturalWidth || width, width) + 40;
                                  const canvasHeight = Math.max(fixedImg.naturalHeight || height, height) + 40;
                                  canvas.width = canvasWidth;
                                  canvas.height = canvasHeight;
                                  
                                  const ctx = canvas.getContext('2d', { 
                                    willReadFrequently: false,
                                    alpha: false // No transparency - ensures solid rendering
                                  });
                                  
                                  if (!ctx) {
                                    document.body.removeChild(tempContainer);
                                    URL.revokeObjectURL(svgUrl);
                                    URL.revokeObjectURL(fixedSvgUrl);
                                    resolve();
                                    return;
                                  }
                                  
                                  // Fill entire canvas with white background FIRST
                                  ctx.fillStyle = '#ffffff';
                                  ctx.fillRect(0, 0, canvas.width, canvas.height);
                                  
                                  // Set rendering quality
                                  ctx.imageSmoothingEnabled = true;
                                  ctx.imageSmoothingQuality = 'high';
                                  
                                  // Draw the SVG image centered with padding
                                  const drawX = 20;
                                  const drawY = 20;
                                  const drawWidth = fixedImg.naturalWidth || width;
                                  const drawHeight = fixedImg.naturalHeight || height;
                                  
                                  // Draw the image
                                  ctx.drawImage(fixedImg, drawX, drawY, drawWidth, drawHeight);
                                  
                                  // Wait for canvas to fully render
                                  setTimeout(() => {
                                    try {
                                      // Verify canvas has visible content by sampling multiple areas
                                      const sampleAreas = [
                                        { x: drawX, y: drawY, w: Math.min(100, drawWidth), h: Math.min(100, drawHeight) },
                                        { x: drawX + drawWidth / 2, y: drawY + drawHeight / 2, w: Math.min(100, drawWidth), h: Math.min(100, drawHeight) },
                                        { x: drawX + drawWidth - 100, y: drawY + drawHeight - 100, w: 100, h: 100 }
                                      ];
                                      
                                      let hasVisibleContent = false;
                                      for (const area of sampleAreas) {
                                        if (area.x >= 0 && area.y >= 0 && area.x + area.w <= canvas.width && area.y + area.h <= canvas.height) {
                                          const imageData = ctx.getImageData(area.x, area.y, area.w, area.h);
                                          const hasNonWhite = imageData.data.some((val, idx) => {
                                            if (idx % 4 === 3) return false; // Skip alpha channel
                                            return val < 240; // Not white (allow some tolerance)
                                          });
                                          if (hasNonWhite) {
                                            hasVisibleContent = true;
                                            break;
                                          }
                                        }
                                      }
                                      
                                      if (!hasVisibleContent) {
                                        console.warn('Canvas appears to have no visible content after rendering');
                                        // Try rendering again without size constraints
                                        ctx.clearRect(0, 0, canvas.width, canvas.height);
                                        ctx.fillStyle = '#ffffff';
                                        ctx.fillRect(0, 0, canvas.width, canvas.height);
                                        ctx.drawImage(fixedImg, drawX, drawY);
                                      }
                                      
                                      // Use the actual drawn image dimensions (excluding padding)
                                      // The canvas has padding (20px on each side), but we want just the image
                                      const actualImageWidth = drawWidth;
                                      const actualImageHeight = drawHeight;
                                      
                                      // Always extract just the image area (no padding) - this prevents overlapping
                                      const trimmedCanvas = document.createElement('canvas');
                                      trimmedCanvas.width = actualImageWidth;
                                      trimmedCanvas.height = actualImageHeight;
                                      const trimmedCtx = trimmedCanvas.getContext('2d', { alpha: false });
                                      
                                      if (!trimmedCtx) {
                                        document.body.removeChild(tempContainer);
                                        URL.revokeObjectURL(svgUrl);
                                        URL.revokeObjectURL(fixedSvgUrl);
                                        resolve();
                                        return;
                                      }
                                      
                                      trimmedCtx.fillStyle = '#ffffff';
                                      trimmedCtx.fillRect(0, 0, trimmedCanvas.width, trimmedCanvas.height);
                                      trimmedCtx.imageSmoothingEnabled = true;
                                      trimmedCtx.imageSmoothingQuality = 'high';
                                      
                                      // Extract just the image area (excluding padding)
                                      trimmedCtx.drawImage(
                                        canvas,
                                        drawX, drawY, actualImageWidth, actualImageHeight,
                                        0, 0, actualImageWidth, actualImageHeight
                                      );
                                      
                                      // Get the trimmed image data URL
                                      const trimmedDataUrl = trimmedCanvas.toDataURL('image/png', 1.0);
                                      
                                      // Create image element with proper styling to prevent overlap
                                      const imgElement = document.createElement('img');
                                      imgElement.src = trimmedDataUrl;
                                      imgElement.alt = 'Mermaid Diagram';
                                      imgElement.style.maxWidth = '100%';
                                      imgElement.style.width = 'auto';
                                      imgElement.style.height = 'auto';
                                      imgElement.style.display = 'block';
                                      imgElement.style.margin = '12pt auto';
                                      imgElement.style.clear = 'both';
                                      imgElement.style.pageBreakInside = 'avoid';
                                      imgElement.style.breakInside = 'avoid';
                                      imgElement.style.objectFit = 'contain';
                                      
                                      const wrapper = document.createElement('div');
                                      wrapper.style.textAlign = 'center';
                                      wrapper.style.display = 'block';
                                      wrapper.style.clear = 'both';
                                      wrapper.style.width = '100%';
                                      // Reduce margin for diagrams - check if previous sibling is a heading
                                      const prevSibling = container.previousElementSibling;
                                      const isAfterHeading = prevSibling && (prevSibling.tagName === 'H2' || prevSibling.tagName === 'H3' || prevSibling.tagName === 'H4');
                                      
                                      // Check if this is System Architecture (first h2 after title)
                                      const isFirstHeading = prevSibling && prevSibling.tagName === 'H2' && 
                                        (prevSibling.previousElementSibling === null || 
                                         prevSibling.previousElementSibling.tagName === 'H1' ||
                                         (prevSibling.previousElementSibling.tagName === 'DIV' && prevSibling.previousElementSibling.classList.contains('document-title')));
                                      
                                      // Check if this is Multi-Agent Coordination
                                      const isMultiAgentCoordination = prevSibling && 
                                        (prevSibling.textContent?.includes('Multi-Agent Coordination') ||
                                         prevSibling.getAttribute('data-multi-agent-coordination') === 'true');
                                      
                                      wrapper.style.margin = isAfterHeading ? '6pt 0' : '12pt 0';
                                      wrapper.style.marginTop = isAfterHeading ? '6pt' : '12pt';
                                      wrapper.style.marginBottom = '24pt'; // Extra space to prevent overlap
                                      
                                      // Always set page break rules for diagrams after headings
                                      if (isAfterHeading) {
                                        wrapper.className = 'heading-diagram-wrapper';
                                        
                                        if (isFirstHeading) {
                                          // System Architecture - make it larger and ensure it stays on same page
                                          wrapper.setAttribute('data-system-architecture', 'true');
                                          wrapper.style.maxHeight = '900pt';
                                          wrapper.style.pageBreakBefore = 'avoid';
                                          wrapper.style.breakBefore = 'avoid';
                                          wrapper.style.pageBreakAfter = 'auto';
                                          wrapper.style.breakAfter = 'auto';
                                          wrapper.style.pageBreakInside = 'avoid';
                                          wrapper.style.breakInside = 'avoid';
                                        } else if (isMultiAgentCoordination) {
                                          // Multi-Agent Coordination - no page break after
                                          wrapper.setAttribute('data-multi-agent-coordination', 'true');
                                          wrapper.style.maxHeight = '800pt';
                                          wrapper.style.pageBreakBefore = 'avoid';
                                          wrapper.style.breakBefore = 'avoid';
                                          wrapper.style.pageBreakAfter = 'avoid';
                                          wrapper.style.breakAfter = 'avoid';
                                          wrapper.style.pageBreakInside = 'avoid';
                                          wrapper.style.breakInside = 'avoid';
                                        } else {
                                          wrapper.style.maxHeight = '800pt';
                                          wrapper.style.pageBreakBefore = 'avoid';
                                          wrapper.style.breakBefore = 'avoid';
                                          wrapper.style.pageBreakInside = 'avoid';
                                          wrapper.style.breakInside = 'avoid';
                                        }
                                      } else {
                                        // Default page break rules for non-heading diagrams
                                        wrapper.style.pageBreakBefore = 'auto';
                                        wrapper.style.pageBreakAfter = 'auto';
                                        wrapper.style.pageBreakInside = 'avoid';
                                        wrapper.style.breakInside = 'avoid';
                                      }
                                      wrapper.appendChild(imgElement);
                                      
                                      if (container.parentNode) {
                                        container.parentNode.replaceChild(wrapper, container);
                                      } else {
                                        container.replaceWith(wrapper);
                                      }
                                      
                                      // Clean up
                                      if (tempContainer.parentNode) {
                                        document.body.removeChild(tempContainer);
                                      }
                                      URL.revokeObjectURL(svgUrl);
                                      URL.revokeObjectURL(fixedSvgUrl);
                                      resolve();
                                    } catch (e) {
                                      console.warn('Error in final canvas conversion:', e);
                                      if (tempContainer.parentNode) {
                                        document.body.removeChild(tempContainer);
                                      }
                                      URL.revokeObjectURL(svgUrl);
                                      URL.revokeObjectURL(fixedSvgUrl);
                                      resolve();
                                    }
                                  }, 800); // Longer wait for canvas rendering
                                } catch (e) {
                                  console.warn('Error creating canvas:', e);
                                  if (tempContainer.parentNode) {
                                    document.body.removeChild(tempContainer);
                                  }
                                  URL.revokeObjectURL(svgUrl);
                                  URL.revokeObjectURL(fixedSvgUrl);
                                  resolve();
                                }
                              }, 500); // Wait for image to fully load
                            };
                            
                            fixedImg.onerror = () => {
                              // Fallback to original image
                              document.body.removeChild(tempContainer);
                              URL.revokeObjectURL(fixedSvgUrl);
                              URL.revokeObjectURL(svgUrl);
                              resolve();
                            };
                            
                            fixedImg.src = fixedSvgUrl;
                          } else {
                            // No SVG found, use original approach
                            document.body.removeChild(tempContainer);
                            URL.revokeObjectURL(svgUrl);
                            resolve();
                          }
                        } catch (e) {
                          console.warn('Error verifying SVG text:', e);
                          if (tempContainer.parentNode) {
                            document.body.removeChild(tempContainer);
                          }
                          URL.revokeObjectURL(svgUrl);
                          resolve();
                        }
                      }, 500); // Longer wait for SVG to fully render in DOM
                    } catch (e) {
                      console.warn('Error setting up SVG rendering:', e);
                      URL.revokeObjectURL(svgUrl);
                      resolve();
                    }
                  }, 200); // Initial delay to ensure image is loaded
                } catch (e) {
                  console.warn('Error in image onload handler:', e);
                  URL.revokeObjectURL(svgUrl);
                  resolve();
                }
              };
              
              img.onerror = (err) => {
                console.warn('Error loading SVG image:', err);
                URL.revokeObjectURL(svgUrl);
                resolve();
              };
              
              // Set crossOrigin to allow canvas rendering
              img.crossOrigin = 'anonymous';
              img.src = svgUrl;
            });
          } catch (e) {
            console.warn('Error processing Mermaid diagram:', e);
          }
        });

        await Promise.all(mermaidPromises);
      };

      await convertMermaidForPDF();

      // Convert images to base64
      const images = clonedContent.querySelectorAll('img');
      const imagePromises = Array.from(images).map(async (img) => {
        const src = img.getAttribute('src');
        if (!src || src.startsWith('data:')) {
          return;
        }

        const imgElement = document.querySelector(`img[src="${CSS.escape(src)}"]`) as HTMLImageElement;
        if (!imgElement) {
          return;
        }

        const canvas = document.createElement('canvas');
        const ctx = canvas.getContext('2d');
        if (!ctx) {
          return;
        }

        const tempImg = new Image();
        tempImg.crossOrigin = 'anonymous';

        return new Promise<void>((resolve) => {
          tempImg.onload = () => {
            try {
              canvas.width = tempImg.naturalWidth || tempImg.width;
              canvas.height = tempImg.naturalHeight || tempImg.height;
              ctx.drawImage(tempImg, 0, 0);
              const dataUrl = canvas.toDataURL('image/png');
              img.setAttribute('src', dataUrl);
            } catch (e) {
              console.warn('Error converting image:', e);
            }
            resolve();
          };

          tempImg.onerror = () => resolve();
          tempImg.src = imgElement.src;
        });
      });

      await Promise.all(imagePromises);

      // IMPORTANT: Wrap headings and diagrams AFTER Mermaid conversion
      // This ensures we're wrapping the actual image divs, not the original mermaid elements
      const wrapHeadingsWithDiagrams = () => {
        const headings = Array.from(clonedContent.querySelectorAll('h2, h3, h4'));
        
        headings.forEach((heading) => {
          // Skip if heading is already in a wrapper
          if (heading.parentElement?.classList.contains('heading-diagram-wrapper')) {
            return;
          }
          
          // Find the next sibling that is a diagram
          let nextSibling = heading.nextElementSibling;
          const elementsToWrap: Element[] = [];
          
          // Skip empty paragraphs and collect them
          while (nextSibling && nextSibling.tagName === 'P' && (!nextSibling.textContent || nextSibling.textContent.trim() === '')) {
            elementsToWrap.push(nextSibling);
            nextSibling = nextSibling.nextElementSibling;
          }
          
          // Check if next sibling is a diagram (div with image or mermaid)
          if (nextSibling) {
            const hasImage = nextSibling.querySelector && nextSibling.querySelector('img');
            const hasCenteredStyle = nextSibling.getAttribute('style')?.includes('text-align: center');
            const hasSvg = nextSibling.querySelector && nextSibling.querySelector('svg');
            const isMermaid = nextSibling.classList.contains('mermaid') || 
                            (nextSibling.tagName === 'PRE' && nextSibling.classList.contains('mermaid'));
            
            const isDiagram = 
              (nextSibling.tagName === 'DIV' && (hasImage || hasCenteredStyle || hasSvg)) ||
              isMermaid;
            
          if (isDiagram) {
            // Check if this is System Architecture (first h2)
            const isFirstHeading = heading.tagName === 'H2' && 
              (heading.previousElementSibling === null || 
               heading.previousElementSibling.tagName === 'H1' ||
               (heading.previousElementSibling.tagName === 'DIV' && heading.previousElementSibling.classList.contains('document-title')));
            
            // Check if this is Multi-Agent Coordination
            const isMultiAgentCoordination = heading.textContent?.includes('Multi-Agent Coordination') || 
              heading.getAttribute('data-multi-agent-coordination') === 'true';
            
            // Create a wrapper container with class for easier CSS targeting
            const wrapper = document.createElement('div');
            wrapper.className = 'heading-diagram-wrapper';
            
            // Add data attributes for CSS targeting
            if (isFirstHeading) {
              wrapper.setAttribute('data-system-architecture', 'true');
            }
            if (isMultiAgentCoordination) {
              wrapper.setAttribute('data-multi-agent-coordination', 'true');
            }
            
            // Set styles based on heading type
            if (isFirstHeading) {
              wrapper.setAttribute('style', 'page-break-inside: avoid !important; break-inside: avoid !important; page-break-before: avoid !important; break-before: avoid !important; page-break-after: auto !important; break-after: auto !important; display: block !important; width: 100% !important; max-height: 900pt !important;');
            } else if (isMultiAgentCoordination) {
              wrapper.setAttribute('style', 'page-break-inside: avoid !important; break-inside: avoid !important; page-break-before: avoid !important; break-before: avoid !important; page-break-after: avoid !important; break-after: avoid !important; display: block !important; width: 100% !important; max-height: 800pt !important;');
            } else {
              wrapper.setAttribute('style', 'page-break-inside: avoid !important; break-inside: avoid !important; display: block !important; width: 100% !important; max-height: 800pt !important;');
            }
            
            // Get parent and insert wrapper before heading
            const parent = heading.parentNode;
            if (parent) {
              parent.insertBefore(wrapper, heading);
              
              // Move heading into wrapper
              wrapper.appendChild(heading);
              
              // Move empty paragraphs into wrapper
              elementsToWrap.forEach(element => {
                if (element.parentNode) {
                  wrapper.appendChild(element);
                }
              });
              
              // Move the diagram into wrapper
              if (nextSibling && nextSibling.parentNode) {
                wrapper.appendChild(nextSibling);
              }
            }
          }
          }
        });
      };
      
      wrapHeadingsWithDiagrams();

      // Create a temporary window for printing
      const printWindow = window.open('', '_blank');
      if (!printWindow) {
        alert('Please allow popups to download PDF');
        if (button) {
          button.disabled = false;
          button.textContent = originalText;
        }
        return;
      }

      printWindow.document.write(`
        <!DOCTYPE html>
        <html>
        <head>
          <title>${title}</title>
          <style>
            @page {
              size: letter;
              margin: 0.75in;
            }
            /* Prevent page breaks between headings and diagrams - AGGRESSIVE */
            h2, h3, h4 {
              page-break-after: avoid !important;
              break-after: avoid !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
            }
            /* Prevent page breaks on diagrams that follow headings - FORCE TOGETHER */
            h2 + pre.mermaid, h2 + .mermaid, h2 + div[style*="text-align: center"],
            h3 + pre.mermaid, h3 + .mermaid, h3 + div[style*="text-align: center"],
            h4 + pre.mermaid, h4 + .mermaid, h4 + div[style*="text-align: center"] {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
              page-break-after: auto !important;
            }
            h2 + p + pre.mermaid, h2 + p + div[style*="text-align: center"],
            h3 + p + pre.mermaid, h3 + p + div[style*="text-align: center"],
            h4 + p + pre.mermaid, h4 + p + div[style*="text-align: center"] {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
            }
            /* Keep heading and diagram together - ALL VARIATIONS */
            h2 + pre, h2 + div, h2 + p + pre, h2 + p + div,
            h3 + pre, h3 + div, h3 + p + pre, h3 + p + div,
            h4 + pre, h4 + div, h4 + p + pre, h4 + p + div {
              page-break-before: avoid !important;
              break-before: avoid !important;
            }
            /* Wrap heading and following diagram in a container to keep together */
            h2, h3, h4 {
              display: block;
              width: 100%;
            }
            /* Style for wrapper divs that contain headings and diagrams */
            .heading-diagram-wrapper,
            div[style*="page-break-inside: avoid"],
            div[style*="break-inside: avoid"] {
              page-break-inside: avoid !important;
              break-inside: avoid !important;
              display: block !important;
              width: 100% !important;
              overflow: visible !important;
            }
            
            /* Special handling for System Architecture wrapper - no page breaks - AGGRESSIVE */
            .heading-diagram-wrapper[data-system-architecture="true"] {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-after: auto !important;
              break-after: auto !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
              max-height: 900pt !important;
            }
            
            /* Ensure System Architecture heading inside wrapper has no page breaks */
            .heading-diagram-wrapper[data-system-architecture="true"] h2 {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-after: avoid !important;
              break-after: avoid !important;
            }
            
            /* Ensure System Architecture diagram inside wrapper has no page breaks */
            .heading-diagram-wrapper[data-system-architecture="true"] img,
            .heading-diagram-wrapper[data-system-architecture="true"] svg,
            .heading-diagram-wrapper[data-system-architecture="true"] div {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
            }
            
            /* Prevent page break after Multi-Agent Coordination heading and diagram */
            h2:nth-of-type(4),
            h2[data-multi-agent-coordination="true"] {
              page-break-after: avoid !important;
              break-after: avoid !important;
            }
            
            h2:nth-of-type(4) + div[style*="text-align: center"],
            h2:nth-of-type(4) + pre.mermaid,
            h2[data-multi-agent-coordination="true"] + div[style*="text-align: center"],
            h2[data-multi-agent-coordination="true"] + pre.mermaid {
              page-break-after: avoid !important;
              break-after: avoid !important;
            }
            
            /* Wrapper for Multi-Agent Coordination - no page break after */
            .heading-diagram-wrapper[data-multi-agent-coordination="true"] {
              page-break-after: avoid !important;
              break-after: avoid !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
            }
            .heading-diagram-wrapper h2,
            .heading-diagram-wrapper h3,
            .heading-diagram-wrapper h4,
            div[style*="page-break-inside: avoid"] h2,
            div[style*="page-break-inside: avoid"] h3,
            div[style*="page-break-inside: avoid"] h4,
            div[style*="break-inside: avoid"] h2,
            div[style*="break-inside: avoid"] h3,
            div[style*="break-inside: avoid"] h4 {
              page-break-after: avoid !important;
              break-after: avoid !important;
              margin-bottom: 6pt !important;
            }
            .heading-diagram-wrapper div,
            .heading-diagram-wrapper img,
            .heading-diagram-wrapper pre,
            div[style*="page-break-inside: avoid"] div,
            div[style*="page-break-inside: avoid"] img,
            div[style*="page-break-inside: avoid"] pre,
            div[style*="break-inside: avoid"] div,
            div[style*="break-inside: avoid"] img,
            div[style*="break-inside: avoid"] pre {
              page-break-before: avoid !important;
              break-before: avoid !important;
              margin-top: 6pt !important;
            }
            /* Ensure diagrams in wrappers don't exceed page height - CRITICAL */
            .heading-diagram-wrapper img,
            div[style*="page-break-inside: avoid"] img,
            div[style*="break-inside: avoid"] img {
              max-height: 300pt !important;
              max-width: 100% !important;
              height: auto !important;
              width: auto !important;
              object-fit: contain !important;
              display: block !important;
            }
            /* Make wrapper itself smaller if needed */
            .heading-diagram-wrapper {
              max-height: 400pt !important;
              overflow: visible !important;
            }
            /* Prevent orphans and widows for headings */
            h2, h3, h4 {
              orphans: 3;
              widows: 3;
            }
            /* Force diagrams to stay with their headings - especially System Architecture */
            h2 ~ div[style*="text-align: center"]:first-of-type,
            h3 ~ div[style*="text-align: center"]:first-of-type,
            h4 ~ div[style*="text-align: center"]:first-of-type {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-after: auto !important;
              max-height: 800pt !important;
              overflow: visible !important;
            }
            /* Ensure images in diagrams don't cause breaks and scale to fit - larger for readability */
            h2 ~ div img, h3 ~ div img, h4 ~ div img,
            h2 + div img, h3 + div img, h4 + div img,
            h2 + p + div img, h3 + p + div img, h4 + p + div img {
              page-break-inside: avoid !important;
              break-inside: avoid !important;
              page-break-before: avoid !important;
              break-before: avoid !important;
              max-height: 800pt !important;
              max-width: 100% !important;
              height: auto !important;
              object-fit: contain !important;
            }
            
            /* Special handling for first diagram after System Architecture heading */
            h2:first-of-type + div[style*="text-align: center"] img,
            h2:first-of-type + div[style*="text-align: center"] svg {
              max-height: 900pt !important;
              page-break-before: avoid !important;
              break-before: avoid !important;
            }
            
            /* Prevent diagram overlap - ensure each diagram wrapper has proper spacing */
            div[style*="text-align: center"] {
              page-break-before: auto !important;
              page-break-after: auto !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
              margin-bottom: 24pt !important;
              margin-top: 12pt !important;
              clear: both !important;
              display: block !important;
            }
            /* Make sure pre.mermaid elements also scale - larger for readability */
            h2 + pre.mermaid, h3 + pre.mermaid, h4 + pre.mermaid,
            h2 + p + pre.mermaid, h3 + p + pre.mermaid, h4 + p + pre.mermaid {
              max-height: 800pt !important;
              overflow: visible !important;
            }
            
            /* Special handling for System Architecture - keep it with heading, no page breaks */
            h2:first-of-type {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-after: avoid !important;
              break-after: avoid !important;
            }
            h2:first-of-type + div[style*="text-align: center"],
            h2:first-of-type + pre.mermaid {
              page-break-before: avoid !important;
              break-before: avoid !important;
              page-break-inside: avoid !important;
              break-inside: avoid !important;
              page-break-after: auto !important;
              break-after: auto !important;
              max-height: 900pt !important;
            }
            
            /* Prevent page break after System Architecture diagram */
            h2:first-of-type + div[style*="text-align: center"] ~ *:first-of-type {
              page-break-before: auto !important;
              break-before: auto !important;
            }
            /* Hide all buttons in print */
            button, .downloadButtons, [aria-label*="Download"] {
              display: none !important;
              visibility: hidden !important;
            }
            body {
              font-family: Arial, sans-serif;
              font-size: 11pt;
              line-height: 1.5;
              color: #000;
              margin: 0;
              padding: 20px;
            }
            .document-title {
              font-size: 28pt;
              font-weight: bold;
              margin-bottom: 24pt;
              padding-bottom: 12pt;
              border-bottom: 2px solid #000;
              text-align: center;
            }
            h1 { font-size: 24pt; font-weight: bold; margin-top: 24pt; margin-bottom: 12pt; }
            h2 { font-size: 18pt; font-weight: bold; margin-top: 18pt; margin-bottom: 6pt; }
            h3 { font-size: 14pt; font-weight: bold; margin-top: 14pt; margin-bottom: 6pt; }
            h4 { font-size: 12pt; font-weight: bold; margin-top: 12pt; margin-bottom: 6pt; }
            p { margin: 6pt 0; }
            /* Remove margin from empty paragraphs or paragraphs between headings and diagrams */
            h2 + p:empty, h3 + p:empty, h4 + p:empty {
              margin: 0 !important;
              height: 0 !important;
              display: none !important;
            }
            /* Reduce spacing when paragraph is between heading and diagram */
            h2 + p + pre.mermaid, h2 + p + .mermaid,
            h3 + p + pre.mermaid, h3 + p + .mermaid,
            h4 + p + pre.mermaid, h4 + p + .mermaid {
              margin-top: 6pt !important;
            }
            h2 + p + div[style*="text-align: center"],
            h3 + p + div[style*="text-align: center"],
            h4 + p + div[style*="text-align: center"] {
              margin-top: 6pt !important;
            }
            ul, ol { margin: 12pt 0; padding-left: 30pt; }
            li { margin: 3pt 0; }
            pre { background: #f5f5f5; border: 1px solid #ddd; padding: 10pt; margin: 12pt 0; font-family: monospace; font-size: 10pt; white-space: pre-wrap; }
            /* Reduce margin on pre elements that follow headings */
            h2 + pre, h3 + pre, h4 + pre {
              margin-top: 6pt !important;
            }
            h2 + p + pre, h3 + p + pre, h4 + p + pre {
              margin-top: 6pt !important;
            }
            code { background: #f5f5f5; padding: 2pt 4pt; font-family: monospace; font-size: 10pt; }
            table { border-collapse: collapse; width: 100%; margin: 12pt 0; }
            th, td { border: 1px solid #ddd; padding: 8pt; text-align: left; }
            th { background: #f0f0f0; font-weight: bold; }
            img { max-width: 100%; height: auto; display: block; margin: 12pt auto; page-break-inside: avoid; break-inside: avoid; clear: both; }
            /* Ensure diagrams don't overlap - add proper spacing */
            div[style*="text-align: center"] {
              page-break-inside: avoid !important;
              break-inside: avoid !important;
              margin: 12pt 0 !important;
              clear: both !important;
              display: block !important;
            }
            /* Reduce spacing between headings and following diagrams/images */
            h2 + img, h2 + div img, h2 + div[style*="text-align: center"],
            h3 + img, h3 + div img, h3 + div[style*="text-align: center"],
            h4 + img, h4 + div img, h4 + div[style*="text-align: center"] {
              margin-top: 6pt !important;
            }
            /* Reduce spacing for Mermaid diagrams that follow headings */
            h2 ~ div[style*="text-align: center"] img,
            h3 ~ div[style*="text-align: center"] img,
            h4 ~ div[style*="text-align: center"] img {
              margin-top: 6pt !important;
            }
            /* Reduce margin on diagram wrapper divs that follow headings */
            h2 + div[style*="text-align: center"],
            h3 + div[style*="text-align: center"],
            h4 + div[style*="text-align: center"] {
              margin-top: 6pt !important;
              margin-bottom: 6pt !important;
            }
            /* Reduce spacing for any div containing images after headings */
            h2 + div, h3 + div, h4 + div {
              margin-top: 6pt !important;
            }
            /* Reduce spacing for Mermaid diagrams (pre.mermaid or .mermaid) after headings */
            h2 + pre.mermaid, h2 + .mermaid,
            h3 + pre.mermaid, h3 + .mermaid,
            h4 + pre.mermaid, h4 + .mermaid {
              margin-top: 6pt !important;
              margin-bottom: 6pt !important;
            }
            /* Also handle when there's a paragraph between heading and mermaid */
            h2 + p + pre.mermaid, h2 + p + .mermaid,
            h3 + p + pre.mermaid, h3 + p + .mermaid,
            h4 + p + pre.mermaid, h4 + p + .mermaid {
              margin-top: 6pt !important;
            }
            /* Reduce spacing for wrapper divs containing Mermaid diagrams */
            h2 + div:has(pre.mermaid), h2 + div:has(.mermaid),
            h3 + div:has(pre.mermaid), h3 + div:has(.mermaid),
            h4 + div:has(pre.mermaid), h4 + div:has(.mermaid) {
              margin-top: 6pt !important;
            }
            /* Fallback for browsers that don't support :has() - target divs with mermaid content */
            h2 ~ div[style*="text-align: center"]:first-of-type,
            h3 ~ div[style*="text-align: center"]:first-of-type,
            h4 ~ div[style*="text-align: center"]:first-of-type {
              margin-top: 6pt !important;
            }
          </style>
        </head>
        <body>
          <div class="document-title">${title}</div>
          ${clonedContent.innerHTML}
        </body>
        </html>
      `);

      printWindow.document.close();
      
      // Wait for all content (including images) to fully load before printing
      const waitForContent = () => {
        const images = printWindow.document.querySelectorAll('img');
        let imagesLoaded = 0;
        const totalImages = images.length;
        
        if (totalImages === 0) {
          // No images, can print immediately
          setTimeout(() => {
            printWindow.print();
            printWindow.close();
            if (button) {
              button.disabled = false;
              button.textContent = originalText;
            }
          }, 500);
          return;
        }
        
        // Wait for all images to load
        images.forEach((img) => {
          const imgElement = img as HTMLImageElement;
          if (imgElement.complete) {
            imagesLoaded++;
            if (imagesLoaded === totalImages) {
              // All images loaded, wait a bit more for rendering, then print
              setTimeout(() => {
                printWindow.print();
                printWindow.close();
                if (button) {
                  button.disabled = false;
                  button.textContent = originalText;
                }
              }, 1000); // Longer delay to ensure everything is rendered
            }
          } else {
            imgElement.onload = () => {
              imagesLoaded++;
              if (imagesLoaded === totalImages) {
                setTimeout(() => {
                  printWindow.print();
                  printWindow.close();
                  if (button) {
                    button.disabled = false;
                    button.textContent = originalText;
                  }
                }, 1000);
              }
            };
            imgElement.onerror = () => {
              imagesLoaded++;
              if (imagesLoaded === totalImages) {
                setTimeout(() => {
                  printWindow.print();
                  printWindow.close();
                  if (button) {
                    button.disabled = false;
                    button.textContent = originalText;
                  }
                }, 1000);
              }
            };
          }
        });
        
        // Fallback timeout in case images don't load
        setTimeout(() => {
          if (printWindow && !printWindow.closed) {
            printWindow.print();
            printWindow.close();
            if (button) {
              button.disabled = false;
              button.textContent = originalText;
            }
          }
        }, 3000); // Maximum 3 second wait
      };
      
      // Wait for window to be ready
      printWindow.addEventListener('load', () => {
        setTimeout(waitForContent, 300);
      });
      
      // Fallback if load event doesn't fire
      setTimeout(waitForContent, 500);
    } catch (error) {
      console.error('Error downloading PDF:', error);
      alert('Error generating PDF. Please try again.');
      if (button) {
        button.disabled = false;
        button.textContent = originalText;
      }
    }
  };

  const handleDownloadDOCX = async () => {
    // Show loading indicator
    const originalText = 'Download DOCX';
    const button = document.querySelector('[aria-label="Download as DOCX"]') as HTMLButtonElement;
    if (button) {
      button.disabled = true;
      button.textContent = 'Processing...';
    }

    try {

      // Get the main content area
      const content = document.querySelector('article') || document.querySelector('main') || document.body;
      
      if (!content) {
        alert('Could not find content to export');
        if (button) {
          button.disabled = false;
          button.textContent = originalText;
        }
        return;
      }

      // Clone the content to avoid modifying the original
      const clonedContent = content.cloneNode(true) as HTMLElement;
      
      // Remove unwanted elements (including download buttons)
      const elementsToRemove = clonedContent.querySelectorAll('script, style, .downloadButtons, nav, aside, footer, header, .navbar, .diagramDownload, button[aria-label*="Download"]');
      elementsToRemove.forEach(el => el.remove());
      
      // Convert images to base64 data URIs for embedding in DOCX
      const convertImageToBase64 = async (img: HTMLImageElement): Promise<string | null> => {
        return new Promise((resolve) => {
          const src = img.getAttribute('src');
          if (!src) {
            resolve(null);
            return;
          }

          if (src.startsWith('data:')) {
            resolve(src);
            return;
          }

          // Try to find the original image in the document
          const imgElement = document.querySelector(`img[src="${CSS.escape(src)}"]`) as HTMLImageElement;
          if (!imgElement) {
            resolve(null);
            return;
          }

          const canvas = document.createElement('canvas');
          const ctx = canvas.getContext('2d');
          if (!ctx) {
            resolve(null);
            return;
          }

          const tempImg = new Image();
          tempImg.crossOrigin = 'anonymous';

          tempImg.onload = () => {
            try {
              canvas.width = tempImg.naturalWidth || tempImg.width;
              canvas.height = tempImg.naturalHeight || tempImg.height;
              ctx.drawImage(tempImg, 0, 0);
              const dataUrl = canvas.toDataURL('image/png');
              resolve(dataUrl);
            } catch (e) {
              console.warn('Error converting image:', e);
              resolve(null);
            }
          };

          tempImg.onerror = () => {
            resolve(null);
          };

          tempImg.src = imgElement.src;
        });
      };

      // Convert all images
      const images = clonedContent.querySelectorAll('img');
      const imagePromises = Array.from(images).map(async (img) => {
        const dataUrl = await convertImageToBase64(img as HTMLImageElement);
        if (dataUrl) {
          img.setAttribute('src', dataUrl);
        }
      });

      // Convert Mermaid diagrams to SVG for DOCX (Word supports SVG)
      // STEP 1: Export Mermaid diagrams as SVG first, then embed in DOCX
      const convertMermaidDiagrams = async () => {
        // First, find the original Mermaid diagrams in the document (not cloned)
        const originalMermaidContainers = document.querySelectorAll('.mermaid, pre.mermaid');
        
        const mermaidPromises = Array.from(originalMermaidContainers).map(async (originalContainer, index) => {
          try {
            // Find the corresponding container in cloned content
            const clonedContainers = clonedContent.querySelectorAll('.mermaid, pre.mermaid');
            const container = clonedContainers[index] as HTMLElement;
            if (!container) {
              return;
            }

            // STEP 1: Find the SVG inside the original mermaid container (from the actual DOM)
            const originalSvg = originalContainer.querySelector('svg') as SVGElement;
            if (!originalSvg) {
              console.warn('No SVG found in Mermaid container');
              return;
            }

            // STEP 2: Get SVG dimensions from the original
            let width = originalSvg.clientWidth || originalSvg.getBoundingClientRect().width || 800;
            let height = originalSvg.clientHeight || originalSvg.getBoundingClientRect().height || 600;
            
            // If dimensions are 0, try getBBox
            if (width === 0 || height === 0) {
              try {
                const bbox = originalSvg.getBBox();
                width = bbox.width || 800;
                height = bbox.height || 600;
              } catch (e) {
                width = 800;
                height = 600;
              }
            }

            // STEP 3: Clone SVG from original (not cloned content) to get actual rendered version
            const clonedSvg = originalSvg.cloneNode(true) as SVGElement;
            
            // STEP 4: Process text elements in DOM to ensure they're black
            const textElements = clonedSvg.querySelectorAll('text, tspan');
            textElements.forEach((textEl) => {
              const textElement = textEl as SVGTextElement;
              textElement.setAttribute('fill', '#000000');
              textElement.style.fill = '#000000';
              textElement.style.color = '#000000';
              textElement.removeAttribute('class');
            });
            
            // STEP 5: Ensure SVG has proper attributes for Word compatibility
            clonedSvg.setAttribute('width', width.toString());
            clonedSvg.setAttribute('height', height.toString());
              clonedSvg.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
              clonedSvg.setAttribute('xmlns:xlink', 'http://www.w3.org/1999/xlink');
            
            // Ensure viewBox is set for proper scaling
            if (!clonedSvg.getAttribute('viewBox')) {
              try {
                const bbox = originalSvg.getBBox();
                if (bbox.width > 0 && bbox.height > 0) {
              clonedSvg.setAttribute('viewBox', `${bbox.x} ${bbox.y} ${bbox.width} ${bbox.height}`);
                } else {
                  clonedSvg.setAttribute('viewBox', `0 0 ${width} ${height}`);
                }
              } catch (e) {
                clonedSvg.setAttribute('viewBox', `0 0 ${width} ${height}`);
              }
            }

            // STEP 6: Generate SVG as string (EXPORT AS SVG)
            const svgString = new XMLSerializer().serializeToString(clonedSvg);
            
            // STEP 7: Process SVG string for Word compatibility
            // Fix colors for better visibility in Word documents
            let processedSvg = svgString
              // Fix text elements - ensure all text is black
              .replace(/<text([^>]*)>/gi, (match, attrs) => {
                let newAttrs = attrs.replace(/fill="[^"]*"/gi, '');
                newAttrs += ' fill="#000000"';
                return `<text${newAttrs}>`;
              })
              .replace(/<tspan([^>]*)>/gi, (match, attrs) => {
                let newAttrs = attrs.replace(/fill="[^"]*"/gi, '');
                newAttrs += ' fill="#000000"';
                return `<tspan${newAttrs}>`;
              })
              // Replace dark background colors with white
              .replace(/fill="#1e1e1e"/gi, 'fill="#ffffff"')
              .replace(/fill="#2d2d2d"/gi, 'fill="#ffffff"')
              .replace(/fill="#212121"/gi, 'fill="#ffffff"')
              .replace(/fill="rgb\(30,\s*30,\s*30\)"/gi, 'fill="#ffffff"')
              .replace(/fill="rgb\(45,\s*45,\s*45\)"/gi, 'fill="#ffffff"')
              // Fix white text on dark backgrounds
              .replace(/fill="#ffffff"/g, (match, offset, str) => {
                const before = str.substring(Math.max(0, offset - 200), offset);
                if (before.includes('<text') || before.includes('<tspan')) {
                  return 'fill="#000000"';
                }
                return match;
              })
              // Fix stroke colors
              .replace(/stroke="#1e1e1e"/gi, 'stroke="#000000"')
              .replace(/stroke="#2d2d2d"/gi, 'stroke="#000000"')
              .replace(/stroke="rgb\(30,\s*30,\s*30\)"/gi, 'stroke="#000000"')
              .replace(/stroke="rgb\(45,\s*45,\s*45\)"/gi, 'stroke="#000000"')
              // Fix background colors
              .replace(/background-color:#1e1e1e/gi, 'background-color:#ffffff')
              .replace(/background-color:#2d2d2d/gi, 'background-color:#ffffff');
            
            // STEP 8: Add CSS style block to ensure text is black
            if (!processedSvg.includes('<style')) {
              processedSvg = processedSvg.replace(
                /(<svg[^>]*>)/i,
                `$1<style type="text/css"><![CDATA[
                  text, tspan { fill: #000000 !important; }
                ]]></style>`
              );
            }

            // STEP 9: Create a wrapper div with the SVG embedded directly
            // Word 2016+ supports inline SVG when properly formatted
            const wrapper = document.createElement('div');
            wrapper.style.textAlign = 'center';
            wrapper.style.margin = '12pt 0';
            wrapper.style.maxWidth = '100%';
            wrapper.style.display = 'block';
            
            // Insert the processed SVG string directly into the wrapper
            // This is the SVG export - we're embedding it as-is
            wrapper.innerHTML = processedSvg;
            
            // STEP 10: Get the SVG element from the wrapper and ensure proper styling
            const svgElement = wrapper.querySelector('svg') as SVGElement;
            if (svgElement) {
              // Ensure SVG has proper dimensions and attributes for Word compatibility
              svgElement.setAttribute('width', width.toString());
              svgElement.setAttribute('height', height.toString());
              svgElement.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
              svgElement.setAttribute('xmlns:xlink', 'http://www.w3.org/1999/xlink');
              
              // Set inline styles for Word compatibility
              svgElement.style.maxWidth = '100%';
              svgElement.style.width = width + 'px';
              svgElement.style.height = 'auto';
              svgElement.style.display = 'block';
              svgElement.style.margin = '0 auto';
              
              // Ensure viewBox is set for proper scaling
              if (!svgElement.getAttribute('viewBox')) {
                try {
                  const bbox = originalSvg.getBBox();
                  if (bbox.width > 0 && bbox.height > 0) {
                    svgElement.setAttribute('viewBox', `${bbox.x} ${bbox.y} ${bbox.width} ${bbox.height}`);
                  } else {
                    svgElement.setAttribute('viewBox', `0 0 ${width} ${height}`);
            }
          } catch (e) {
                  svgElement.setAttribute('viewBox', `0 0 ${width} ${height}`);
                }
              }
            }
            
            // STEP 11: Replace the mermaid container with the SVG wrapper
            // The SVG is now embedded in the DOCX HTML as inline SVG
            if (container.parentNode) {
              container.parentNode.replaceChild(wrapper, container);
            } else {
              container.replaceWith(wrapper);
            }
          } catch (e) {
            console.warn('Error processing Mermaid diagram for DOCX:', e);
          }
        });

        await Promise.all(mermaidPromises);
      };

      await convertMermaidDiagrams();

      // Wait for all image conversions to complete
      await Promise.all(imagePromises);
      
      // Clean up code blocks - convert to plain text
      const codeBlocks = clonedContent.querySelectorAll('pre, code');
      codeBlocks.forEach(block => {
        const text = block.textContent || '';
        const span = document.createElement('span');
        span.textContent = text;
        block.parentNode?.replaceChild(span, block);
      });

      // Create a Word-compatible HTML document with proper structure
      const htmlContent = `
        <!DOCTYPE html>
        <html xmlns:v="urn:schemas-microsoft-com:vml"
              xmlns:o="urn:schemas-microsoft-com:office:office"
              xmlns:w="urn:schemas-microsoft-com:office:word"
              xmlns:m="http://schemas.microsoft.com/office/2004/12/omml"
              xmlns="http://www.w3.org/TR/REC-html40">
        <head>
          <meta charset="UTF-8">
          <meta name="ProgId" content="Word.Document">
          <meta name="Generator" content="Microsoft Word">
          <meta name="Originator" content="Microsoft Word">
          <title>${title}</title>
          <!--[if gte mso 9]>
          <xml>
            <w:WordDocument>
              <w:View>Print</w:View>
              <w:Zoom>90</w:Zoom>
              <w:DoNotOptimizeForBrowser/>
            </w:WordDocument>
          </xml>
          <![endif]-->
          <style>
            @page {
              size: 8.5in 11in;
              margin: 1in;
            }
            body {
              font-family: 'Calibri', 'Arial', sans-serif;
              font-size: 11pt;
              line-height: 1.5;
              color: #000;
              margin: 0;
              padding: 20px;
            }
            h1 {
              font-size: 24pt;
              font-weight: bold;
              margin-top: 24pt;
              margin-bottom: 12pt;
              color: #000;
            }
            h2 {
              font-size: 18pt;
              font-weight: bold;
              margin-top: 18pt;
              margin-bottom: 10pt;
              color: #000;
            }
            h3 {
              font-size: 14pt;
              font-weight: bold;
              margin-top: 14pt;
              margin-bottom: 8pt;
              color: #000;
            }
            h4 {
              font-size: 12pt;
              font-weight: bold;
              margin-top: 12pt;
              margin-bottom: 6pt;
              color: #000;
            }
            p {
              margin: 6pt 0;
              text-align: justify;
            }
            ul, ol {
              margin: 12pt 0;
              padding-left: 30pt;
            }
            li {
              margin: 3pt 0;
            }
            pre {
              background: #f5f5f5;
              border: 1px solid #ddd;
              padding: 10pt;
              margin: 12pt 0;
              font-family: 'Courier New', monospace;
              font-size: 10pt;
              white-space: pre-wrap;
              word-wrap: break-word;
            }
            code {
              background: #f5f5f5;
              padding: 2pt 4pt;
              font-family: 'Courier New', monospace;
              font-size: 10pt;
            }
            table {
              border-collapse: collapse;
              width: 100%;
              margin: 12pt 0;
            }
            th, td {
              border: 1px solid #ddd;
              padding: 8pt;
              text-align: left;
            }
            th {
              background: #f0f0f0;
              font-weight: bold;
            }
            img {
              max-width: 100%;
              height: auto;
              display: block;
              margin: 12pt auto;
            }
            svg {
              max-width: 100%;
              height: auto;
              display: block;
              margin: 12pt auto;
            }
            hr {
              border: none;
              border-top: 1px solid #ddd;
              margin: 24pt 0;
            }
          </style>
        </head>
        <body>
          <h1 style="font-size: 28pt; font-weight: bold; margin-bottom: 24pt; padding-bottom: 12pt; border-bottom: 2px solid #000; text-align: center;">${title}</h1>
          ${clonedContent.innerHTML}
        </body>
        </html>
      `;

      // Convert HTML to DOCX using html-docx-js
      const blob = await asBlob(htmlContent);
      
      // Create download link with .docx extension
      const url = URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `${filename}.docx`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);

      // Restore button
      if (button) {
        button.disabled = false;
        button.textContent = originalText;
      }
    } catch (error) {
      console.error('Error downloading DOCX:', error);
      alert('Error generating document. Please try again.');
      if (button) {
        button.disabled = false;
        button.textContent = originalText;
      }
    }
  };

  return (
    <div className={styles.downloadButtons}>
      <button 
        className={styles.downloadButton}
        onClick={handleDownloadPDF}
        aria-label="Download as PDF"
      >
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
          <polyline points="7 10 12 15 17 10" />
          <line x1="12" y1="15" x2="12" y2="3" />
        </svg>
        Download PDF
      </button>
      <button 
        className={styles.downloadButton}
        onClick={handleDownloadDOCX}
        aria-label="Download as DOCX"
      >
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
          <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
          <polyline points="7 10 12 15 17 10" />
          <line x1="12" y1="15" x2="12" y2="3" />
        </svg>
        Download DOCX
      </button>
    </div>
  );
}

