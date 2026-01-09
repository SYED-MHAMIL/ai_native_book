import React from 'react';
import Layout from '@theme/Layout';
import TestShadcn from '../components/TestShadcn';

export default function TestPage() {
  return (
    <Layout title="Shadcn Components Test" description="Test page for shadcn components">
      <main className="container mx-auto">
        <TestShadcn />
      </main>
    </Layout>
  );
}