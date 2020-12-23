package us.ihmc.robotics.robotDescription.loaders.urdf.items;

import java.util.Collections;
import java.util.List;

import javax.xml.bind.annotation.XmlAttribute;

public class URDFMass implements URDFItem
{
   private String value;

   @XmlAttribute(name = "value")
   public void setValue(String value)
   {
      this.value = value;
   }

   public String getValue()
   {
      return value;
   }

   @Override
   public String getContentAsString()
   {
      return format("[value: %s]", value);
   }

   @Override
   public String toString()
   {
      return itemToString();
   }

   @Override
   public List<URDFFilenameHolder> getFilenameHolders()
   {
      return Collections.emptyList();
   }
}
