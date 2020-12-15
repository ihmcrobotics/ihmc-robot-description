package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DMatrix;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;

public class LinkDescription
{
   private String name;

   private double mass;
   private final Vector3D centerOfMassOffset = new Vector3D();
   private final DMatrixRMaj momentOfInertia = new DMatrixRMaj(3, 3);

   private final Vector3D principalMomentsOfInertia = new Vector3D();
   private final RotationMatrix principalAxesRotation = new RotationMatrix();

   private LinkGraphicsDescription linkGraphics;
   private final List<CollisionMeshDescription> collisionMeshes = new ArrayList<>();

   public LinkDescription(String name)
   {
      this.name = name;
   }

   public LinkDescription(LinkDescription other)
   {
      name = other.name;
      mass = other.mass;
      centerOfMassOffset.set(other.centerOfMassOffset);
      momentOfInertia.set(other.momentOfInertia);
      principalMomentsOfInertia.set(other.principalMomentsOfInertia);
      principalAxesRotation.set(other.principalAxesRotation);

      linkGraphics = new LinkGraphicsDescription();
      linkGraphics.combine(other.linkGraphics);

      other.collisionMeshes.forEach(collisionMesh -> collisionMeshes.add(collisionMesh.copy()));
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public String getName()
   {
      return name;
   }

   public LinkGraphicsDescription getLinkGraphics()
   {
      return linkGraphics;
   }

   public void setLinkGraphics(LinkGraphicsDescription linkGraphics)
   {
      this.linkGraphics = linkGraphics;
   }

   public List<CollisionMeshDescription> getCollisionMeshes()
   {
      return collisionMeshes;
   }

   public void addCollisionMesh(CollisionMeshDescription collisionMesh)
   {
      collisionMeshes.add(collisionMesh);
   }

   public double getMass()
   {
      return mass;
   }

   public void setMass(double mass)
   {
      if (mass < 0.0)
         throw new RuntimeException("mass < 0.0");
      this.mass = mass;
   }

   public void getCenterOfMassOffset(Tuple3DBasics centerOfMassOffsetToPack)
   {
      centerOfMassOffsetToPack.set(centerOfMassOffset);
   }

   public Vector3D getCenterOfMassOffset()
   {
      return centerOfMassOffset;
   }

   public void setCenterOfMassOffset(Tuple3DReadOnly centerOfMassOffset)
   {
      this.centerOfMassOffset.set(centerOfMassOffset);
   }

   public void setCenterOfMassOffset(double xOffset, double yOffset, double zOffset)
   {
      centerOfMassOffset.set(xOffset, yOffset, zOffset);
   }

   public void setMomentOfInertia(DMatrix momentOfInertia)
   {
      this.momentOfInertia.set(momentOfInertia);
   }

   public void setMomentOfInertia(Matrix3DReadOnly momentOfInertia)
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            this.momentOfInertia.set(i, j, momentOfInertia.getElement(i, j));
         }
      }
   }

   public Matrix3D getMomentOfInertiaCopy()
   {
      Matrix3D momentOfInertia = new Matrix3D();

      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
         {
            momentOfInertia.setElement(i, j, this.momentOfInertia.get(i, j));
         }
      }

      return momentOfInertia;
   }

   public void setMassAndRadiiOfGyration(double mass, double radiusOfGyrationX, double radiusOfGyrationY, double radiusOfGyrationZ)
   {
      this.mass = mass;

      double Ixx = mass * (radiusOfGyrationY * radiusOfGyrationY + radiusOfGyrationZ * radiusOfGyrationZ);
      double Iyy = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationZ * radiusOfGyrationZ);
      double Izz = mass * (radiusOfGyrationX * radiusOfGyrationX + radiusOfGyrationY * radiusOfGyrationY);

      setMomentOfInertia(Ixx, Iyy, Izz);
   }

   public void getMomentOfInertia(DMatrix momentOfInertiaToPack)
   {
      momentOfInertiaToPack.set(momentOfInertia);
   }

   public DMatrixRMaj getMomentOfInertia()
   {
      return momentOfInertia;
   }

   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      momentOfInertia.zero();
      momentOfInertia.set(0, 0, Ixx);
      momentOfInertia.set(1, 1, Iyy);
      momentOfInertia.set(2, 2, Izz);

   }

   // ////////// Graphics from Mass Properties Here ///////////////////////

   /**
    * Adds an ellipsoid representing the mass and inertia of the link at its center of mass. This
    * ellipsoid has a default matte black appearance.
    */
   public void addEllipsoidFromMassProperties()
   {
      addEllipsoidFromMassProperties(null);
   }

   /**
    * Adds a coordinate system representation at the center of mass of this link. The axis of this
    * system have the given length.
    *
    * @param length length in meters of each arm/axis on the coordinate system.
    */
   public void addCoordinateSystemToCOM(double length)
   {
      linkGraphics.identity();

      Vector3D comOffset = new Vector3D();
      getCenterOfMassOffset(comOffset);

      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.addCoordinateSystem(length);

      linkGraphics.identity();
   }

   public void addEllipsoidFromMassProperties2(AppearanceDefinition appearance)
   {
      computePrincipalMomentsOfInertia();

      Vector3D inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      ArrayList<Vector3D> inertiaEllipsoidAxes = new ArrayList<>();

      Vector3D e1 = new Vector3D();
      principalAxesRotation.getColumn(0, e1);
      e1.normalize();
      e1.scale(inertiaEllipsoidRadii.getX());
      inertiaEllipsoidAxes.add(e1);
      Vector3D e2 = new Vector3D();
      principalAxesRotation.getColumn(1, e2);
      e2.normalize();
      e2.scale(inertiaEllipsoidRadii.getY());
      inertiaEllipsoidAxes.add(e2);
      Vector3D e3 = new Vector3D();
      principalAxesRotation.getColumn(2, e3);
      e3.normalize();
      e3.scale(inertiaEllipsoidRadii.getZ());
      inertiaEllipsoidAxes.add(e3);
      Vector3D e4 = new Vector3D(e1);
      e4.negate();
      inertiaEllipsoidAxes.add(e4);
      Vector3D e5 = new Vector3D(e2);
      e5.negate();
      inertiaEllipsoidAxes.add(e5);
      Vector3D e6 = new Vector3D(e3);
      e6.negate();
      inertiaEllipsoidAxes.add(e6);

      double vertexSize = 0.01 * inertiaEllipsoidRadii.length();

      Vector3D comOffset = new Vector3D();
      getCenterOfMassOffset(comOffset);

      for (Vector3D vector : inertiaEllipsoidAxes)
      {
         linkGraphics.identity();
         linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
         linkGraphics.translate(vector);
         linkGraphics.addCube(vertexSize, vertexSize, vertexSize, appearance);
      }

      ArrayList<Point3D> inertiaOctahedronVertices = new ArrayList<>();

      Point3D p1 = new Point3D(e1);
      inertiaOctahedronVertices.add(p1);
      Point3D p2 = new Point3D(e2);
      inertiaOctahedronVertices.add(p2);
      Point3D p3 = new Point3D(e4);
      inertiaOctahedronVertices.add(p3);
      Point3D p4 = new Point3D(e5);
      inertiaOctahedronVertices.add(p4);
      Point3D p5 = new Point3D(e3);
      inertiaOctahedronVertices.add(p5);
      Point3D p6 = new Point3D(e6);
      inertiaOctahedronVertices.add(p6);

      ArrayList<Point3D> face1 = new ArrayList<>();
      face1.add(p1);
      face1.add(p5);
      face1.add(p4);
      ArrayList<Point3D> face2 = new ArrayList<>();
      face2.add(p4);
      face2.add(p5);
      face2.add(p3);
      ArrayList<Point3D> face3 = new ArrayList<>();
      face3.add(p3);
      face3.add(p5);
      face3.add(p2);
      ArrayList<Point3D> face4 = new ArrayList<>();
      face4.add(p2);
      face4.add(p5);
      face4.add(p1);

      ArrayList<Point3D> face5 = new ArrayList<>();
      face5.add(p4);
      face5.add(p6);
      face5.add(p1);
      ArrayList<Point3D> face6 = new ArrayList<>();
      face6.add(p3);
      face6.add(p6);
      face6.add(p4);
      ArrayList<Point3D> face7 = new ArrayList<>();
      face7.add(p2);
      face7.add(p6);
      face7.add(p3);
      ArrayList<Point3D> face8 = new ArrayList<>();
      face8.add(p1);
      face8.add(p6);
      face8.add(p2);

      linkGraphics.identity();
      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.addPolygon(face1, appearance);
      linkGraphics.addPolygon(face2, appearance);
      linkGraphics.addPolygon(face3, appearance);
      linkGraphics.addPolygon(face4, appearance);
      linkGraphics.addPolygon(face5, appearance);
      linkGraphics.addPolygon(face6, appearance);
      linkGraphics.addPolygon(face7, appearance);
      linkGraphics.addPolygon(face8, appearance);

      //    linkGraphics.identity();
      //    linkGraphics.translate(comOffset.x, comOffset.y, comOffset.z);
      //    linkGraphics.rotate(principalAxesRotation);
      //    linkGraphics.addEllipsoid(inertiaEllipsoidRadii.x, inertiaEllipsoidRadii.y, inertiaEllipsoidRadii.z, appearance);
      //    linkGraphics.identity();
   }

   /**
    * Adds an ellipsoid representing the mass and inertia of the link at its center of mass with the
    * specified appearance.
    *
    * @param appearance Appearance to be used with the ellipsoid. See {@link YoAppearance YoAppearance}
    *                   for implementations.
    */
   public void addEllipsoidFromMassProperties(AppearanceDefinition appearance)
   {
      computePrincipalMomentsOfInertia();

      Vector3D inertiaEllipsoidRadii = InertiaTools.getInertiaEllipsoidRadii(principalMomentsOfInertia, mass);

      if (appearance == null)
         appearance = YoAppearance.Black();

      Vector3D comOffset = new Vector3D();
      getCenterOfMassOffset(comOffset);

      linkGraphics.identity();
      linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
      linkGraphics.rotate(principalAxesRotation);
      linkGraphics.addEllipsoid(inertiaEllipsoidRadii.getX(), inertiaEllipsoidRadii.getY(), inertiaEllipsoidRadii.getZ(), appearance);
      linkGraphics.identity();
   }

   /**
    * Adds an box representing the mass and inertia of the link at its center of mass with the
    * specified appearance. Specifically, mimics the code from Gazebo to debug SDF loader See
    * https://bitbucket.org/osrf/gazebo/src/0709b57a8a3a8abce3c67e992e5c6a5c24c8d84a/gazebo/rendering/COMVisual.cc?at=default
    *
    * @param appearance Appearance to be used with the ellipsoid. See {@link YoAppearance YoAppearance}
    *                   for implementations.
    */
   public void addBoxFromMassProperties(AppearanceDefinition appearance)
   {
      Vector3D comOffset = new Vector3D();
      getCenterOfMassOffset(comOffset);

      if (mass <= 0 || momentOfInertia.get(0, 0) <= 0 || momentOfInertia.get(1, 1) <= 0 || momentOfInertia.get(2, 2) <= 0
            || momentOfInertia.get(0, 0) + momentOfInertia.get(1, 1) <= momentOfInertia.get(2, 2)
            || momentOfInertia.get(1, 1) + momentOfInertia.get(2, 2) <= momentOfInertia.get(0, 0)
            || momentOfInertia.get(0, 0) + momentOfInertia.get(2, 2) <= momentOfInertia.get(1, 1))
      {
         System.err.println(getName() + " has unrealistic inertia");
      }
      else
      {
         linkGraphics.identity();
         linkGraphics.translate(comOffset.getX(), comOffset.getY(), comOffset.getZ());
         double lx = Math.sqrt(6.0 * (momentOfInertia.get(2, 2) + momentOfInertia.get(1, 1) - momentOfInertia.get(0, 0)) / mass);
         double ly = Math.sqrt(6.0 * (momentOfInertia.get(2, 2) + momentOfInertia.get(0, 0) - momentOfInertia.get(1, 1)) / mass);
         double lz = Math.sqrt(6.0 * (momentOfInertia.get(0, 0) + momentOfInertia.get(1, 1) - momentOfInertia.get(2, 2)) / mass);
         linkGraphics.translate(0.0, 0.0, -lz / 2.0);
         linkGraphics.addCube(lx, ly, lz, appearance);
         linkGraphics.identity();
      }
   }

   public void computePrincipalMomentsOfInertia()
   {
      InertiaTools.computePrincipalMomentsOfInertia(momentOfInertia, principalAxesRotation, principalMomentsOfInertia);
   }

   public void scale(double factor, double massScalePower, boolean scaleInertia)
   {
      // Center of mass offset scales with the scaling factor
      centerOfMassOffset.scale(factor);

      // Mass scales with factor^massScalePower. massScalePower is 3 when considering constant density

      if (scaleInertia)
      {
         mass = Math.pow(factor, massScalePower) * mass;
         // The components of the inertia matrix are defined with int(r^2 dm). So they scale factor ^ (2 + massScalePower)
         double inertiaScale = Math.pow(factor, massScalePower + 2);
         CommonOps_DDRM.scale(inertiaScale, momentOfInertia);
         computePrincipalMomentsOfInertia();
      }

      if (linkGraphics != null)
      {
         linkGraphics.preScale(factor);
      }

      if (collisionMeshes != null)
      {
         for (int i = 0; i < collisionMeshes.size(); i++)
         {
            collisionMeshes.get(i).scale(factor);
         }
      }
   }

   public LinkDescription copy()
   {
      return new LinkDescription(this);
   }

   @Override
   public String toString()
   {
      return "Link: " + name;
   }
}
