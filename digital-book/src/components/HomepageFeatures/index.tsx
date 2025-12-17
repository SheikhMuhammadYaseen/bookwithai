import type { ReactNode } from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  image: string;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Comprehensive Curriculum',
    image: '/img/comprehensive-curriculum.png',
    description: (
      <p>
        Master foundational to advanced topics in Physical AI and Humanoid Robotics
        through structured, progressive lessons.
      </p>
    ),
  },
  {
    title: 'Hands-On Interactive Learning',
    image: '/img/interactive-learning.png',
    description: (
      <p>
        Build skills with simulations, live code editors, quizzes, and practical
        projects in real-world robotics scenarios.
      </p>
    ),
  },
  {
    title: 'Deep Dive into Humanoid Robotics',
    image: '/img/humanoid-robotics.png',
    description: (
      <p>
        Explore sensors, actuators, control systems, AI perception, locomotion, and
        embodiment in cutting-edge humanoid designs.
      </p>
    ),
  },
];

function Feature({ title, image, description }: FeatureItem) {
  return (
    <div className={clsx('col col--4', styles.featureItem)}>
      <div className={styles.featureImageWrapper}>
        <img
          src={image}
          alt={title}
          className={styles.featureImage}
        />
      </div>
      <div className={styles.featureText}>
        <Heading as="h3">{title}</Heading>
        {description}
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props) => (
            <Feature key={props.title} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}