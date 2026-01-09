import React, { useState } from 'react';

const Tabs = ({ children, defaultValue, onValueChange, className = '' }) => {
  const [activeTab, setActiveTab] = useState(defaultValue);

  const handleTabChange = (value) => {
    setActiveTab(value);
    if (onValueChange) {
      onValueChange(value);
    }
  };

  const tabList = React.Children.toArray(children).find(child => child.type?.name === 'TabsList');
  const tabContent = React.Children.toArray(children).find(child => child.type?.name === 'TabsContent');

  return (
    <div className={className}>
      {tabList && React.cloneElement(tabList, {
        activeTab,
        onTabChange: handleTabChange,
        children: React.Children.map(tabList.props.children, child =>
          React.cloneElement(child, { activeTab, onClick: () => handleTabChange(child.props.value) })
        )
      })}
      {tabContent && React.cloneElement(tabContent, {
        activeTab,
        children: React.Children.map(tabContent.props.children, child =>
          React.cloneElement(child, { activeTab })
        )
      })}
    </div>
  );
};

const TabsList = ({ children, className = '' }) => {
  return (
    <div className={`flex space-x-1 p-1 bg-gray-100 rounded-lg ${className}`}>
      {children}
    </div>
  );
};

const TabsTrigger = ({ value, children, activeTab, onClick, className = '' }) => {
  const isActive = activeTab === value;
  return (
    <button
      className={`px-4 py-2 text-sm font-medium rounded-md transition-colors ${
        isActive
          ? 'bg-white text-gray-900 shadow-sm'
          : 'text-gray-600 hover:text-gray-900 hover:bg-gray-50'
      } ${className}`}
      onClick={onClick}
    >
      {children}
    </button>
  );
};

const TabsContent = ({ children, activeTab, className = '' }) => {
  return (
    <div className={`mt-4 ${className}`}>
      {React.Children.map(children, child => {
        if (child.props.value === activeTab) {
          return child;
        }
        return null;
      })}
    </div>
  );
};

const TabContent = ({ value, children, activeTab }) => {
  return activeTab === value ? <div>{children}</div> : null;
};

export { Tabs, TabsList, TabsTrigger, TabsContent, TabContent };