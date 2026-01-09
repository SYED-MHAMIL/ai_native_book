import React from 'react';
import Layout from '@theme/Layout';
import { Card, CardContent } from '../ui/card';
import { Breadcrumb } from '../navigation/Navigation';

const DocumentationLayout = ({
  title,
  description,
  children,
  breadcrumbs = [],
  toc = [],
  showToc = true,
  className = ''
}) => {
  return (
    <Layout title={title} description={description}>
      <div className="min-h-screen bg-gray-50">
        {/* Header */}
        <header className="bg-white shadow-sm border-b">
          <div className="container mx-auto px-4 py-4">
            <Breadcrumb items={breadcrumbs} className="mb-2" />
            <h1 className="text-3xl font-bold text-gray-900">{title}</h1>
            {description && <p className="text-gray-600 mt-2">{description}</p>}
          </div>
        </header>

        <div className="container mx-auto px-4 py-8">
          <div className="flex flex-col lg:flex-row gap-8">
            {/* Main Content */}
            <main className={`flex-1 ${showToc ? 'lg:max-w-3xl' : 'lg:max-w-4xl'}`}>
              <Card className="shadow-sm">
                <CardContent className="p-8">
                  <div className="prose prose-gray max-w-none">
                    {children}
                  </div>
                </CardContent>
              </Card>
            </main>

            {/* Table of Contents */}
            {showToc && toc.length > 0 && (
              <aside className="lg:w-80">
                <Card className="sticky top-8">
                  <CardContent className="p-6">
                    <h3 className="font-semibold text-gray-900 mb-4">Table of Contents</h3>
                    <nav>
                      <ul className="space-y-2">
                        {toc.map((item, index) => (
                          <li key={index}>
                            <a
                              href={item.href}
                              className="text-sm text-gray-600 hover:text-blue-600 hover:underline block py-1"
                            >
                              {item.title}
                            </a>
                          </li>
                        ))}
                      </ul>
                    </nav>
                  </CardContent>
                </Card>
              </aside>
            )}
          </div>
        </div>

        {/* Footer */}
        <footer className="bg-gray-900 text-white py-8 mt-12">
          <div className="container mx-auto px-4 text-center">
            <p>© {new Date().getFullYear()} robot.ai. All rights reserved.</p>
          </div>
        </footer>
      </div>
    </Layout>
  );
};

export default DocumentationLayout;