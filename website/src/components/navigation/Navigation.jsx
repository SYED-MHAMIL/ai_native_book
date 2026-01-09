import React from 'react';
import { Button } from '../ui/button';

const Navigation = ({
  items = [],
  currentPath = '',
  variant = 'horizontal',
  className = ''
}) => {
  const NavigationItem = ({ item, isActive }) => {
    if (item.external) {
      return (
        <a
          href={item.href}
          target="_blank"
          rel="noopener noreferrer"
          className={`px-3 py-2 text-sm font-medium rounded-md transition-colors ${
            isActive
              ? 'bg-blue-100 text-blue-700'
              : 'text-gray-700 hover:text-blue-600 hover:bg-gray-100'
          }`}
        >
          {item.label}
        </a>
      );
    }

    return (
      <a
        href={item.href}
        className={`px-3 py-2 text-sm font-medium rounded-md transition-colors ${
          isActive
            ? 'bg-blue-100 text-blue-700'
            : 'text-gray-700 hover:text-blue-600 hover:bg-gray-100'
        }`}
      >
        {item.label}
      </a>
    );
  };

  if (variant === 'vertical') {
    return (
      <nav className={`bg-white border-r border-gray-200 ${className}`}>
        <div className="p-4">
          <ul className="space-y-2">
            {items.map((item, index) => (
              <li key={index}>
                <NavigationItem
                  item={item}
                  isActive={currentPath === item.href}
                />
              </li>
            ))}
          </ul>
        </div>
      </nav>
    );
  }

  return (
    <nav className={`bg-white border-b border-gray-200 ${className}`}>
      <div className="container mx-auto px-4">
        <div className="flex space-x-8">
          {items.map((item, index) => (
            <NavigationItem
              key={index}
              item={item}
              isActive={currentPath === item.href}
            />
          ))}
        </div>
      </div>
    </nav>
  );
};

const Breadcrumb = ({ items = [], className = '' }) => {
  return (
    <nav className={`flex ${className}`} aria-label="Breadcrumb">
      <ol className="inline-flex items-center space-x-1 md:space-x-2">
        {items.map((item, index) => (
          <li key={index} className="inline-flex items-center">
            {index > 0 && (
              <svg
                className="w-4 h-4 text-gray-400"
                fill="none"
                stroke="currentColor"
                viewBox="0 0 24 24"
              >
                <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5l7 7-7 7" />
              </svg>
            )}
            {item.href ? (
              <a
                href={item.href}
                className={`ml-1 text-sm font-medium ${
                  index === items.length - 1
                    ? 'text-gray-900'
                    : 'text-gray-700 hover:text-blue-600'
                }`}
              >
                {item.label}
              </a>
            ) : (
              <span className="ml-1 text-sm font-medium text-gray-900">
                {item.label}
              </span>
            )}
          </li>
        ))}
      </ol>
    </nav>
  );
};

const Sidebar = ({ items = [], currentPath = '', className = '' }) => {
  return (
    <aside className={`bg-white border-r border-gray-200 h-full ${className}`}>
      <div className="p-4">
        <nav>
          <ul className="space-y-1">
            {items.map((item, index) => (
              <li key={index}>
                <a
                  href={item.href}
                  className={`flex items-center px-3 py-2 text-sm font-medium rounded-md transition-colors ${
                    currentPath === item.href
                      ? 'bg-blue-100 text-blue-700'
                      : 'text-gray-700 hover:text-blue-600 hover:bg-gray-100'
                  }`}
                >
                  {item.icon && <item.icon className="w-4 h-4 mr-2" />}
                  {item.label}
                </a>
              </li>
            ))}
          </ul>
        </nav>
      </div>
    </aside>
  );
};

export { Navigation, Breadcrumb, Sidebar };