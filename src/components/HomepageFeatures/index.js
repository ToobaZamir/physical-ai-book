import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Module 1: ROS 2 Mastery',
    image: require('@site/static/img/ros2.png').default,
    description: (
      <>
        Learn the core architecture, nodes, topics, services, and actions
        to build production-grade humanoid robot packages.
      </>
    ),
  },
  {
    title: 'Module 2: Digital Twins',
    image: require('@site/static/img/digital-twin.png').default,
    description: (
      <>
        Explore Gazebo, Isaac Sim, and Unity simulations to transfer skills
        from virtual models to real humanoid robots.
      </>
    ),
  },
  {
    title: 'Module 3: AI-Robot Brain',
    image: require('@site/static/img/ai-brain.png').default,
    description: (
      <>
        Dive into perception pipelines, reinforcement learning, and
        hardware-accelerated AI for humanoid robots.
      </>
    ),
  },
];

function Feature({image, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center">
        <img
          src={image}
          alt={title}
          className={styles.featureImage}
        />
      </div>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
