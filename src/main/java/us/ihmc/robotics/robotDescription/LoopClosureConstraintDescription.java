package us.ihmc.robotics.robotDescription;

import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * A loop closure constraint can be seen as a passive joint that serves only for closing a kinematic
 * loop.
 * <p>
 * Such constraint is needed to complete mechanisms such as four bar linkages.
 * </p>
 * <p>
 * Like a joint, a constraint is attached as child of another joint in the kinematic tree and it's
 * configuration is defined in the local coordinates of the parent joint.
 * </p>
 */
public class LoopClosureConstraintDescription implements RobotDescriptionNode
{
   private String name;

   private JointDescription parentJoint;
   private final Vector3D offsetFromParentJoint = new Vector3D();

   private LinkDescription link;
   private final Vector3D offsetFromLinkParentJoint = new Vector3D();

   private final Matrix3D constraintForceSubSpace = new Matrix3D();
   private final Matrix3D constraintMomentSubSpace = new Matrix3D();

   private final Vector3D proportionalGains = new Vector3D();
   private final Vector3D derivativeGains = new Vector3D();

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
   public static LoopClosureConstraintDescription createPinConstraintDescription(String name, Tuple3DReadOnly offsetFromParentJoint,
                                                                                 Tuple3DReadOnly offsetFromLinkParentJoint, Vector3DReadOnly axis)
   {
      Matrix3D constraintForceSubSpace = new Matrix3D();
      constraintForceSubSpace.setIdentity();
      Matrix3D constraintMomentSubSpace = matrix3DOrthogonalToVector3D(axis);

      return new LoopClosureConstraintDescription(name, offsetFromParentJoint, offsetFromLinkParentJoint, constraintForceSubSpace, constraintMomentSubSpace);
   }

   /**
    * Creates a new constraint for enforcing a loop closure that has 1 degree of freedom, i.e. the
    * translation along the given axis.
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
    * @param axis                      the axis defining the translation axis to remain unconstrained.
    * @return the description of the constraint.
    */
   public static LoopClosureConstraintDescription createSliderConstraintDescription(String name, Tuple3DReadOnly offsetFromParentJoint,
                                                                                    Tuple3DReadOnly offsetFromLinkParentJoint, Vector3DReadOnly axis)
   {
      Matrix3D constraintForceSubSpace = matrix3DOrthogonalToVector3D(axis);
      Matrix3D constraintMomentSubSpace = new Matrix3D();
      constraintMomentSubSpace.setIdentity();

      return new LoopClosureConstraintDescription(name, offsetFromParentJoint, offsetFromLinkParentJoint, constraintForceSubSpace, constraintMomentSubSpace);
   }

   private static Matrix3D matrix3DOrthogonalToVector3D(Vector3DReadOnly vector3D)
   {
      Vector3D orthogonalA = new Vector3D();
      Vector3D orthogonalB = new Vector3D();

      // Purposefully picking a large tolerance to ensure sanity of the cross-product.
      if (Math.abs(vector3D.getY()) > 0.1 || Math.abs(vector3D.getZ()) > 0.1)
         orthogonalA.set(1.0, 0.0, 0.0);
      else
         orthogonalA.set(0.0, 1.0, 0.0);

      orthogonalB.cross(orthogonalA, vector3D);
      orthogonalB.normalize();
      orthogonalA.cross(vector3D, orthogonalB);
      orthogonalA.normalize();

      Matrix3D orthogonalMatrix = new Matrix3D();
      orthogonalMatrix.setToZero();
      orthogonalMatrix.setColumn(0, orthogonalA);
      orthogonalMatrix.setColumn(1, orthogonalB);
      return orthogonalMatrix;
   }

   /**
    * Creates a generic constraint for enforcing a loop closure which degrees of freedom depends on the
    * given {@code constraintForceSubSpace} and {@code constraintMomentSubSpace}.
    * <p>
    * The constraint will be applied to maintain a relative configuration between the link of the
    * parent joint of this constraint, and the constraint's link.
    * </p>
    * <p>
    * This constraint is general and the sub-space in which it operates has to be provided. Here's a
    * couple examples:
    * <ul>
    * <li>For a constraint that only allows rotation around the z-axis, the matrices defining the
    * sub-space should be as follows:
    * 
    * <pre>
    *                           / 1 0 0 \
    * constraintForceSubSpace = | 0 1 0 |
    *                           \ 0 0 1 /
    *                            / 1 0 0 \
    * constraintMomentSubSpace = | 0 1 0 |
    *                            \ 0 0 0 /
    * </pre>
    * 
    * Note that by having <tt>constraintForceSubSpace</tt> be identity, the entire linear space is
    * constrained, while by having the last row of <tt>constraintMomentSubSpace</tt> be only zeros, the
    * z-axis is not constrained.
    * <li>For a constraint that only allows translation along the y-axis, the matrices defining the
    * sub-space should be as follows:
    * 
    * <pre>
    *                           / 1 0 0 \
    * constraintForceSubSpace = | 0 0 0 |
    *                           \ 0 0 1 /
    *                            / 1 0 0 \
    * constraintMomentSubSpace = | 0 1 0 |
    *                            \ 0 0 1 /
    * </pre>
    * </ul>
    * In other words, these matrices can be seen as selection matrices used for selecting which forces
    * and moments are to be applied.
    * </p>
    * 
    * @param name                      the name of the constraint, {@code YoVariable}s will be created
    *                                  using this name.
    * @param offsetFromParentJoint     the position of the constraint with respect to the parent joint.
    * @param offsetFromLinkParentJoint the position of the constraint with respect to the parent joint
    *                                  of the associated link. Note that the link's parent joint is
    *                                  expected to be different from this constraint's parent joint.
    * @param constraintForceSubSpace   defines the linear part of the sub-space in which the constraint
    *                                  is to be enforced.
    * @param constraintMomentSubSpace  defines the angular part of the sub-space in which the
    *                                  constraint is to be enforced.
    */
   public LoopClosureConstraintDescription(String name, Tuple3DReadOnly offsetFromParentJoint, Tuple3DReadOnly offsetFromLinkParentJoint,
                                           Matrix3DReadOnly constraintForceSubSpace, Matrix3DReadOnly constraintMomentSubSpace)
   {
      this.name = name;
      this.offsetFromParentJoint.set(offsetFromParentJoint);
      this.offsetFromLinkParentJoint.set(offsetFromLinkParentJoint);
      this.constraintForceSubSpace.set(constraintForceSubSpace);
      this.constraintMomentSubSpace.set(constraintMomentSubSpace);
   }

   public void setGains(double proportionalGain, double derivativeGain)
   {
      proportionalGains.set(proportionalGain, proportionalGain, proportionalGain);
      derivativeGains.set(derivativeGain, derivativeGain, derivativeGain);
   }

   /**
    * Sets the gains to use for enforcing this constraint.
    * <p>
    * Note that the gains are applied on the error in the local coordinates of the parent joints.
    * </p>
    * 
    * @param proportionalGains the gains to apply on the position and rotation errors.
    * @param derivativeGains   the gains to apply on the linear and angular velocity errors.
    */
   public void setGains(Tuple3DReadOnly proportionalGains, Tuple3DReadOnly derivativeGains)
   {
      this.proportionalGains.set(proportionalGains);
      this.derivativeGains.set(derivativeGains);
   }

   @Override
   public String getName()
   {
      return name;
   }

   public void setParentJoint(JointDescription parentJoint)
   {
      this.parentJoint = parentJoint;
   }

   public void setOffsetFromParentJoint(Tuple3DReadOnly offset)
   {
      offsetFromParentJoint.set(offset);
   }

   public JointDescription getParentJoint()
   {
      return parentJoint;
   }

   public Vector3DBasics getOffsetFromParentJoint()
   {
      return offsetFromParentJoint;
   }

   public Vector3DBasics getOffsetFromLinkParentJoint()
   {
      return offsetFromLinkParentJoint;
   }

   public LinkDescription getLink()
   {
      return link;
   }

   public void setLink(LinkDescription link)
   {
      this.link = link;
   }

   @Override
   public List<JointDescription> getChildrenJoints()
   {
      return Collections.emptyList();
   }

   public Matrix3DBasics getConstraintForceSubSpace()
   {
      return constraintForceSubSpace;
   }

   public Matrix3DBasics getConstraintMomentSubSpace()
   {
      return constraintMomentSubSpace;
   }

   public Vector3DBasics getProportionalGains()
   {
      return proportionalGains;
   }

   public Vector3DBasics getDerivativeGains()
   {
      return derivativeGains;
   }

   @Override
   public void scale(double factor, double massScalePower, List<String> ignoreInertiaScaleJointList)
   {
   }
}
