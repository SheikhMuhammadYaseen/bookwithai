<<<<<<< HEAD
import type { ReactNode } from 'react';
=======
import type {ReactNode} from 'react';
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
<<<<<<< HEAD
  image: string;
=======
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
<<<<<<< HEAD
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
=======
    title: 'Comprehensive Learning',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Explore foundational concepts and advanced topics in Physical AI and
        Humanoid Robotics with our structured and easy-to-follow content.
      </>
    ),
  },
  {
    title: 'Interactive Experience',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Engage with interactive simulations, code examples, and practical
        exercises to reinforce your understanding and apply your knowledge.
      </>
    ),
  },
  {
    title: 'Deep Dive into Robotics',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        From sensors and actuators to advanced control and AI perception, gain
        in-depth insights into building and managing humanoid robots.
      </>
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
    ),
  },
];

<<<<<<< HEAD
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
=======
function Feature({title, Svg, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <Svg className={styles.featureSvg} role="img" />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
      </div>
    </div>
  );
}

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
<<<<<<< HEAD
          {FeatureList.map((props) => (
            <Feature key={props.title} {...props} />
=======
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
          ))}
        </div>
      </div>
    </section>
  );
}
