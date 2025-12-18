import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
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
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started - 5min ⏱️
          </Link>
        </div>
      </div>
    </header>
  );
}

function FeatureList() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          <div className={clsx('col col--4')}>
            <div className="text--center">
              <h3>Physical AI</h3>
            </div>
            <p>Bridge the gap between digital AI models and embodied intelligence.</p>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center">
              <h3>ROS 2 & NVIDIA Isaac</h3>
            </div>
            <p>Learn with industry-standard robotics frameworks and simulation tools.</p>
          </div>
          <div className={clsx('col col--4')}>
            <div className="text--center">
              <h3>Personalized Learning</h3>
            </div>
            <p>Content adapted to your hardware specifications and experience level.</p>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Bridging Digital AI and Embodied Intelligence">
      <HomepageHeader />
      <main>
        <FeatureList />
        <div className={styles.contentSection}>
          <div className="container">
            <div className="row">
              <div className="col col--8 col--offset-2">
                <h2>What You'll Learn</h2>
                <p>
                  This comprehensive textbook covers both theoretical foundations and practical implementation
                  of humanoid robots capable of intelligent interaction with the physical world. You'll explore:
                </p>
                <ul>
                  <li>The transition from digital-only models (LLMs) to Embodied Intelligence</li>
                  <li>How to bridge the gap between "Digital Brain" (AI) and "Physical Body" (Robotics)</li>
                  <li>Physics-informed AI and natural human interaction</li>
                  <li>Use of humanoids as the ultimate form factor for a human-centered world</li>
                </ul>

                <h2>Technical Domains</h2>
                <ul>
                  <li><strong>Robotic Nervous System (ROS 2)</strong>: Nodes, topics, services, actions, and `rclpy`</li>
                  <li><strong>Digital Twins</strong>: Gazebo and Unity for hardware simulation</li>
                  <li><strong>AI-Robot Brain (NVIDIA Isaac)</strong>: Isaac Sim, Isaac ROS, Omniverse, and VSLAM</li>
                  <li><strong>Vision-Language-Action (VLA)</strong>: Integrating LLMs/VLMs with robotic control</li>
                </ul>

                <h2>Key Features</h2>
                <ul>
                  <li>Interactive code blocks with simulation capabilities</li>
                  <li>Hardware-aware content that adapts to your specifications</li>
                  <li>Urdu translations for broader accessibility</li>
                  <li>AI-powered chat assistant with textbook knowledge</li>
                  <li>15-week curriculum with hands-on projects</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      </main>
    </Layout>
  );
}
