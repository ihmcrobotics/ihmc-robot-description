package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class LoopClosurePinConstraintDescription extends LoopClosureConstraintDescription
{
   private final Vector3D axis = new Vector3D();

   /**
    * Creates a new constraint for enforcing a loop closure that has 1 degree of freedom, i.e. the
    * rotation around the given axis.
    * <p>
    * The constraint will be applied to maintain a relative configuration between the link of the
    * parent joint of this constraint, and the constraint's link.
    * </p>
    * 
    * @param name                      the name of the constraint, {@code YoVariable}s will be created
    *                                  using this name.
    * @param offsetFromParentJoint     the position of the constraint with respect to the parent joint.
    * @param offsetFromLinkParentJoint the position of the constraint with respect to the parent joint
    *                                  of the associated link. Note that the link's parent joint is
    *                                  expected to be different from this constraint's parent joint.
    * @param robot                     the robot is used for getting access to its
    * @param axis                      the axis defining the rotation axis to remain unconstrained.
    * @return the description of the constraint.
    */
   public LoopClosurePinConstraintDescription(String name, Tuple3DReadOnly offsetFromParentJoint, Tuple3DReadOnly offsetFromLinkParentJoint,
                                              Vector3DReadOnly axis)
   {
      super(name, offsetFromParentJoint, offsetFromLinkParentJoint);
      setAxis(axis);
   }

   /**
    * Sets the axis around which this constraint is allowed to freely rotate.
    * <p>
    * This method also updates internally the constraint momentum sub-space.
    * </p>
    * 
    * @param axis the axis defining the rotation axis to remain unconstrained.
    */
   public void setAxis(Vector3DReadOnly axis)
   {
      this.axis.set(axis);
      getConstraintForceSubSpace().setIdentity();
      matrix3DOrthogonalToVector3D(axis, getConstraintMomentSubSpace());
   }

   public Vector3DBasics getAxis()
   {
      return axis;
   }
}
