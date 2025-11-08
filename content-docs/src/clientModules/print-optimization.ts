/**
 * Client-side module to optimize page layout for printing
 * Wraps headings and diagrams together to prevent page breaks
 */

function wrapHeadingsWithDiagrams() {
  // Wait for page to be fully loaded
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', wrapHeadingsWithDiagrams);
    return;
  }

  // Find all headings
  const headings = document.querySelectorAll('article h2, article h3, article h4');
  
  headings.forEach((heading) => {
    // Add data attribute for Multi-Agent Coordination
    if (heading.textContent?.includes('Multi-Agent Coordination')) {
      heading.setAttribute('data-multi-agent-coordination', 'true');
    }
    
    // Skip if already wrapped
    if (heading.parentElement?.classList.contains('heading-diagram-wrapper')) {
      return;
    }
    
    // Find the next sibling that is a diagram
    let nextSibling = heading.nextElementSibling;
    const elementsToWrap: Element[] = [];
    
    // Skip empty paragraphs
    while (nextSibling && nextSibling.tagName === 'P' && (!nextSibling.textContent || nextSibling.textContent.trim() === '')) {
      elementsToWrap.push(nextSibling);
      nextSibling = nextSibling.nextElementSibling;
    }
    
    // Check if next sibling is a diagram
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
        // Create wrapper container
        const wrapper = document.createElement('div');
        wrapper.className = 'heading-diagram-wrapper';
        wrapper.setAttribute('style', 'page-break-inside: avoid !important; break-inside: avoid !important; display: block !important; width: 100% !important;');
        
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
}

// Run on page load
wrapHeadingsWithDiagrams();

// Also run when Mermaid diagrams are rendered (they load asynchronously)
if (typeof window !== 'undefined') {
  // Use MutationObserver to catch when Mermaid diagrams are rendered
  const observer = new MutationObserver(() => {
    wrapHeadingsWithDiagrams();
  });
  
  // Observe the article element for changes
  const article = document.querySelector('article');
  if (article) {
    observer.observe(article, {
      childList: true,
      subtree: true
    });
  }
  
  // Also run after a delay to catch late-rendering diagrams
  setTimeout(wrapHeadingsWithDiagrams, 2000);
}

