import React from 'react';
import Layout from '@theme-original/DocItem/Layout';
import Chatbot from '@site/src/components/Chatbot';

export default function DocItemLayout(props) {
  return (
    <>
      <Layout {...props} />
      <Chatbot />
    </>
  );
}