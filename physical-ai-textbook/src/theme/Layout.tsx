import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatWidget from '@components/ChatWidget/ChatWidget';

type LayoutProps = {
  children: React.ReactNode;
  [key: string]: any;
};

export default function Layout(props: LayoutProps): JSX.Element {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatWidget />
    </>
  );
}