import React from 'react';
import { Button } from './ui/button';
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from './ui/card';

const TestShadcn = () => {
  return (
    <div className="container mx-auto py-8">
      <h1 className="text-3xl font-bold mb-6">Shadcn Components Test Page</h1>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-6">
        {/* Card Example */}
        <Card className="w-full">
          <CardHeader>
            <CardTitle>Test Card</CardTitle>
            <CardDescription>This is a test card component</CardDescription>
          </CardHeader>
          <CardContent>
            <p>This card is using the shadcn card component with proper styling.</p>
          </CardContent>
          <CardFooter>
            <Button variant="default">Default Button</Button>
            <Button variant="secondary" className="ml-2">Secondary Button</Button>
            <Button variant="destructive" className="ml-2">Destructive Button</Button>
          </CardFooter>
        </Card>

        {/* More Buttons Example */}
        <Card className="w-full">
          <CardHeader>
            <CardTitle>Button Variants</CardTitle>
          </CardHeader>
          <CardContent className="space-y-4">
            <div className="flex flex-wrap gap-2">
              <Button variant="default">Default</Button>
              <Button variant="secondary">Secondary</Button>
              <Button variant="destructive">Destructive</Button>
              <Button variant="outline">Outline</Button>
              <Button variant="ghost">Ghost</Button>
              <Button variant="link">Link</Button>
            </div>
            <div className="flex flex-wrap gap-2">
              <Button size="sm">Small</Button>
              <Button size="default">Default</Button>
              <Button size="lg">Large</Button>
            </div>
          </CardContent>
        </Card>
      </div>
    </div>
  );
};

export default TestShadcn;