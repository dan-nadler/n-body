
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra;

namespace N_Body__Backend
{
    class Program
    {
        static void Main(string[] args)
        {
            Body b1 = new Body(new[] { 0.0, 0.0, 0.0 }, new[] { 0.0, 0.0, 0.0 }, 10E26, 1, "b1");
            Body b2 = new Body(new[] { 1.0e12, 0.0, 0.0 }, new[] { 0.0, 5.0e6, 0.0 }, 10E24, 1, "b2");
            Body b3 = new Body(new[] { -1.0e12, 0.0, 0.0 }, new[] { 0.0, -5.0e6, 0.0 }, 20E24, 1, "b3");
            Body b4 = new Body(new[] { -1.0e12, -1.0e12, 0.0 }, new[] { -2.50e6, -2.50e6, 0.0 }, 1.5E24, 1, "b4");
            Body[] bodies = { b1, b2, b3, b4 };
            Simulation sim = new Simulation( bodies );
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"C:\Users\Daniel\Documents\positions.csv"))
            {
                for (int t=0; t<=50000; t++)
                {
                    sim.CalculateStep();

                    if (false)
                    {
                        Console.Write((b1.position - b2.position).L2Norm());
                        Console.Write("\n");
                        Console.Write((b1.position - b3.position).L2Norm());
                        Console.Write("\n");
                        Console.Write((b3.position - b2.position).L2Norm());
                        Console.ReadKey();
                        Console.Clear();
                    }
                    foreach (Body b in bodies)
                    {
                        file.Write(b.position[0].ToString() + "," + b.position[1].ToString()+",");
                    }
                    file.Write("\n");
                }
            }
        }
    }

    class Body
    {
        public Vector<double> position;
        public Vector<double> velocity;
        public Vector<double> acceleration;
        public Vector<double> force;
        public double mass;
        public double radius;
        public string name;

        public Body(double[] position, double[] velocity, double mass, double radius, string name)
        {
            this.name = name;
            this.position = Vector<double>.Build.Dense(position);
            this.velocity = Vector<double>.Build.Dense(velocity);
            this.acceleration = Vector<double>.Build.Dense(3);
            this.force = Vector<double>.Build.Dense(3);
            this.mass = mass;
            this.radius = radius;
        }

        public void Reset()
        {
            this.acceleration = Vector<double>.Build.Dense(3);
            this.force = Vector<double>.Build.Dense(3);
        }

        public void ApplyForce()
        {
            // F = m * a
            this.acceleration += this.force / this.mass;
        }

        public void ApplyAcceleration()
        {
            this.velocity += this.acceleration;
        }

        public void ApplyVelocity()
        {
            this.position += this.velocity;
        }

        public void PrintStatus()
        {
            Console.Write("\n");
            Console.Write(name);
            Console.Write("\n");
            Console.Write("position:\n");
            Console.Write(position);
            Console.Write("velocity:\n");
            Console.Write(velocity);
            Console.Write("acceleration:\n");
            Console.Write(acceleration);
            Console.Write("force:\n");
            Console.Write(force);
            Console.Write("mass:");
            Console.Write(mass);
            Console.Write("\n");
            Console.Write("radius:");
            Console.Write(radius);
        }
    }

    class Simulation
    {

        Body[] bodies;
        const double G = 6.67408 * 10e-11;

        public Simulation(Body[] bodies)
        {
            this.bodies = bodies;
        }

        public void CalculateStep()
        {
            for (int i = 0; i < bodies.Count(); i++)
            {
                for (int j = 0; j < i; j++)
                {
                    CalculateForce(bodies[i], bodies[j]);
                }
            }

            for (int i = 0; i < bodies.Count(); i++)
            {
                bodies[i].ApplyForce();
                bodies[i].ApplyAcceleration();
                bodies[i].ApplyVelocity();
                bodies[i].Reset();
            }
        }

        public void CalculateForce(Body b1, Body b2)
        {
            double m1 = b1.mass;
            double m2 = b2.mass;

            Vector<double> p1 = b1.position;
            Vector<double> p2 = b2.position;

            Vector<double> d = p1 - p2;
            Vector<double> r = d/d.L2Norm();

            if (d.L2Norm() <= b1.radius + b2.radius)
            {
                // combine bodies
            }
            else
            {
                // Fg = G * m1 * m2 / r^2
            Vector<double> f = (G * m1 * m2 / (d*d)) * r;
            
            b1.force -= f;
            b2.force += f;
            }
        }
    }
}
