import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div style={{ 
          backgroundColor: 'rgba(255, 255, 255, 0.1)', 
          padding: '1rem', 
          borderRadius: '8px', 
          margin: '1rem 0',
          border: '2px solid rgba(255, 255, 255, 0.3)'
        }}>
          <strong>⚠️ Internal Use Only:</strong> This website is for RoboCon Contractors and Employees only.
        </div>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/getting-started/introduction">
            Get Started - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
      <Layout
        title={`${siteConfig.title}`}
        description="Build intelligent applications for ROBOCON OS powered by ROS 2, Nav 2, and DeepSeek LLM">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
