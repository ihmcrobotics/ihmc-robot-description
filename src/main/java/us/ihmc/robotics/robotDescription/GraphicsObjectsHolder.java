package us.ihmc.robotics.robotDescription;

import java.util.List;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.robotics.robotDescription.collision.CollisionMeshDescription;

public interface GraphicsObjectsHolder
{
   public abstract List<CollisionMeshDescription> getCollisionObjects(String name);

   public abstract Graphics3DObject getGraphicsObject(String name);
}
