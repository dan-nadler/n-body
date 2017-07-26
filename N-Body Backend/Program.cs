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
            List<Body> bodies = new List<Body>();
            int n = 500; // number of bodies
            int T = 500000; // time steps

            // initialization parameters
            double positionScalar = 10e10;
            double velocityScalar = 30e5;
            double noiseLevel = .2;
            Initialization(n, positionScalar, velocityScalar, noiseLevel, bodies, "sphere");

            Simulation sim = new Simulation(bodies);
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"c:\users\daniel\documents\visual studio 2017\Projects\N-Body Backend\N-Body Backend\sphere_positions_detail.csv"))
            {
                file.Write("X,Y,Z,Mass,Time,Body\n");
                for (int t = 0; t <= T; t++) 
                {
                    double avgR = 0;
                    for (int i = 0; i < bodies.Count(); i++)
                    {
                        avgR += bodies[i].position.L2Norm();
                    }
                    avgR /= bodies.Count();

                    if (bodies.Count() <= n/10)
                    {
                        break;
                    }

                    Console.Write("Bodies: " + bodies.Count().ToString() + "; t=" + t.ToString() + "; Avg. Distance: " + avgR / positionScalar + "\r");
                    sim.CalculateStep();

                    if (t % 10 == 0)
                    {
                        int i = 0;
                        foreach (Body b in bodies)
                        {
                            file.Write(
                                (b.position[0] / positionScalar).ToString() + "," //x
                                + (b.position[1] / positionScalar).ToString() + "," //y
                                + (b.position[2] / positionScalar).ToString() + "," //z
                                + b.mass / 10e30 + "," //color
                                + t.ToString() + "," //time
                                + b.id + "\n"); //label
                            i++;
                        }
                    }
                }
            }
        }

        private static void Initialization(int n, double positionScalar, double velocityScalar, double noiseLevel, List<Body> bodies, string type = "toroid")
        {
            Random rnd = new Random();

            double noise = 1.0 / noiseLevel;
            for (int i = 0; i < n; i++)
            {
                double x, y, z, vx, vy, vz;

                if (type == "toroid")
                {
                    x = Math.Sin(Math.PI * i / (n / 2)) + ((rnd.NextDouble() - 0.5) / noise);
                    y = Math.Cos(Math.PI * i / (n / 2)) + ((rnd.NextDouble() - 0.5) / noise);
                    z = ((rnd.NextDouble() - 0.5) / noise);

                    vx = y;
                    vy = -x;
                    vz = 0.0;

                } else if (type == "cube") {

                    x = (rnd.NextDouble() - 0.5) / noise;
                    y = (rnd.NextDouble() - 0.5) / noise;
                    z = (rnd.NextDouble() - 0.5) / noise;

                    vx = (rnd.NextDouble() - 0.5) / noise;
                    vy = (rnd.NextDouble() - 0.5) / noise;
                    vz = (rnd.NextDouble() - 0.5) / noise;

                } else if (type == "sphere") {

                    bool pos = rnd.NextDouble() > 0.5;

                    double r = 1;
                    double theta = rnd.NextDouble() * Math.PI * 1.0;
                    double phi = rnd.NextDouble() * Math.PI * 2.0;

                    double rp = r * Math.Cos(phi);

                    x = rp * Math.Sin(theta);
                    y = r * Math.Sin(phi);
                    z = rp * Math.Cos(theta);

                    vx = 0.0;
                    vy = 0.0;
                    vz = 0.0;

                    // positionScalar *= 5;
                    // velocityScalar *= 10.0;

                } else { // cube 

                    x = (rnd.NextDouble() - 0.5) / noise;
                    y = (rnd.NextDouble() - 0.5) / noise;
                    z = (rnd.NextDouble() - 0.5) / noise;

                    vx = (rnd.NextDouble() - 0.5) / noise;
                    vy = (rnd.NextDouble() - 0.5) / noise;
                    vz = (rnd.NextDouble() - 0.5) / noise;
                }

                Body b = new Body(
                           new[] { x * positionScalar, y * positionScalar, z * positionScalar },
                           new[] { vx * velocityScalar, vy * velocityScalar, vz * velocityScalar },
                           20E30,
                           30e7,
                           "b" + i.ToString(),
                           i
                       );

                bodies.Add(b);
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
        public int id;

        public Body(double[] position, double[] velocity, double mass, double radius, string name, int id)
        {
            this.name = name;
            this.position = Vector<double>.Build.Dense(position);
            this.velocity = Vector<double>.Build.Dense(velocity);
            this.acceleration = Vector<double>.Build.Dense(3);
            this.force = Vector<double>.Build.Dense(3);
            this.mass = mass;
            this.radius = radius;
            this.id = id;
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

        List<Body> bodies;
        const double G = 6.67408 * 10e-11;

        public Simulation(List<Body> bodies)
        {
            this.bodies = bodies;
        }

        public void CalculateStep()
        {
            //for (int i = 0; i < bodies.Count(); i++)
            Parallel.For(0, bodies.Count(), i =>
            {
                //for (int j = 0; j < i; j++)
                Parallel.For(0, i, j =>
                {
                    CalculateForce(bodies[i], bodies[j]);
                });
            });

            Parallel.For(0, bodies.Count(), i =>
            {
                bodies[i].ApplyForce();
                bodies[i].ApplyAcceleration();
                bodies[i].ApplyVelocity();
                bodies[i].Reset();
            });

            CullMergedBodies();
            CullNaNBodies();
        }

        public void CullMergedBodies()
        {
            List<Body> bodiesToRemove = new List<Body>();
            foreach (Body b in bodies)
            {
                if (b.mass == 0)
                {
                    bodiesToRemove.Add(b);
                }
            }

            foreach (Body b in bodiesToRemove)
            {
                bodies.Remove(b);
            }
        }

        public void CullNaNBodies()
        {
            List<Body> bodiesToRemove = new List<Body>();
            foreach (Body b in bodies)
            {
                if (double.IsNaN(b.position.L2Norm()))
                {
                    bodiesToRemove.Add(b);
                }
            }

            foreach (Body b in bodiesToRemove)
            {
                bodies.Remove(b);
            }
        }

        public void CalculateForce(Body b1, Body b2)
        {
            double m1 = b1.mass;
            double m2 = b2.mass;

            Vector<double> p1 = b1.position;
            Vector<double> p2 = b2.position;

            Vector<double> d = p1 - p2;
            Vector<double> r = d / d.L2Norm();

            if (d.L2Norm() <= b1.radius + b2.radius)
            {
                // combine bodies
                double totalMass = b1.mass + b2.mass;
                b1.velocity *= (b1.mass / totalMass);
                b1.velocity += (b2.velocity * (b2.mass/totalMass));

                b1.mass += b2.mass;
                b2.mass = 0;
            }
            else
            {
                // Fg = G * m1 * m2 / r^2
                Vector<double> f = (G * m1 * m2 / (d * d)) * r;

                b1.force -= f;
                b2.force += f;
            }
        }
    }
}
