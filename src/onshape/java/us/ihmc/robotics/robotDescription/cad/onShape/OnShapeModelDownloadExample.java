package us.ihmc.robotics.robotDescription.cad.onShape;

import com.onshape.api.Onshape;
import com.onshape.api.exceptions.OnshapeException;
import com.onshape.api.requests.AssembliesGetAssemblyDefinitionRequest;
import com.onshape.api.responses.*;
import com.onshape.api.types.Blob;
import com.onshape.api.types.OnshapeDocument;
import com.onshape.api.types.WV;
import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commons.thread.ThreadTools;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OnShapeModelDownloadExample
{
   private static final String ACCESS_KEY = "<get from onshape>";
   private static final String SECRET_KEY = "<get from onshape>";
   private static Onshape onshapeClient;

   public static void main(String[] args) throws OnshapeException
   {
      onshapeClient = new Onshape();

      onshapeClient.setAPICredentials(ACCESS_KEY, SECRET_KEY);

      OnshapeDocument nadiaDocument = new OnshapeDocument(
            "https://cad.onshape.com/documents/871126d098857e4aa6cb22f5/w/c8fe7c7ad176f6e266baa3af/e/d745b6ae5c978562f7d9333c");

      AssembliesGetAssemblyDefinitionRequest.Builder assemblyDefinitionBuilder = onshapeClient.assemblies().getAssemblyDefinition().includeMateConnectors(true)
                                                                                              .includeMateFeatures(true);
      AssembliesGetAssemblyDefinitionResponse call = assemblyDefinitionBuilder.call(nadiaDocument);

      List<AssembliesCreateTranslationResponse> colladaTranslations = new ArrayList<>();

      String searchString = "Pelvis";

      Set<String> documentIDs = new HashSet<>();
      for (AssembliesGetAssemblyDefinitionResponseSubAssemblies subAssembly : call.getSubAssemblies())
      {
         for (AssembliesGetAssemblyDefinitionResponseSubAssembliesInstances instance : subAssembly.getInstances())
         {
            System.out.println("Instance: " + instance.name);
         }
      }

      for (AssembliesGetAssemblyDefinitionResponseRootAssemblyInstances instance : call.getRootAssembly().instances)
      {
         if (instance.name.contains(searchString))
         {
            String documentId = instance.documentId;
            if (!documentIDs.contains(documentId))
            {
               System.out.println("Instance: " + instance.name);
               AssembliesCreateTranslationResponse translation = onshapeClient.assemblies().createTranslation().formatName("COLLADA")
                                                                              .call(documentId, WV.Version, instance.documentVersion, instance.elementId);

               colladaTranslations.add(translation);
               documentIDs.add(documentId);
            }
         }
      }

//      for (AssembliesGetAssemblyDefinitionResponseRootAssemblyInstances instances : instancesToDownload)
//      {
//         AssembliesGetAssemblyDefinitionResponse instanceDefinitionResponse = onshapeClient.assemblies().getAssemblyDefinition().includeMateConnectors(true)
//                                                                                         .includeMateFeatures(true).call(instances.getDocumentId(), WVM.Version,
//                                                                                                                         instances.documentVersion,
//                                                                                                                         instances.elementId);
//
//         System.out.println("Instance Definition:" + instanceDefinitionResponse);
//      }

      colladaTranslations.parallelStream().map(OnShapeModelDownloadExample::getBlobForTranslation).filter(Objects::nonNull)
                         .forEach(OnShapeModelDownloadExample::saveBlobToColladaFile);
   }

   private static void saveBlobToColladaFile(ImmutablePair<String, Blob> data)
   {
      Path savePath = Paths.get(System.getProperty("user.home"), "tmp", data.left);

      if (!savePath.getParent().toFile().exists())
      {
         savePath.getParent().toFile().mkdirs();
      }

      FileOutputStream outputStream = null;
      try
      {
         outputStream = new FileOutputStream(savePath.toFile(), false);
         outputStream.write(data.right.getData());
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private static ImmutablePair<String, Blob> getBlobForTranslation(AssembliesCreateTranslationResponse colladaTranslation)
   {
      try
      {
         boolean translationFailed = false;
         if (colladaTranslation != null)
         {
            String requestState = colladaTranslation.getRequestState();

            System.out.println("Request State for " + colladaTranslation.getId() + ": " + requestState);

            if (requestState.toLowerCase().equals("active"))
            {
               boolean translationDoneOrError = false;

               while (!translationDoneOrError)
               {
                  ThreadTools.sleep(3000);
                  TranslationsGetTranslationResponse translationCall = onshapeClient.translations().getTranslation().call(colladaTranslation.getId());

                  String state = translationCall.getRequestState();
                  //                  System.out.println("Translation state: " + state);

                  if (state.toLowerCase().equals("done") || state.toLowerCase().equals("failed"))
                  {
                     if (state.toLowerCase().equals("failed"))
                     {
                        translationFailed = true;
                     }
                     translationDoneOrError = true;
                  }
               }
            }
         }

         String translationHref = null;
         if (!translationFailed)
         {
            translationHref = colladaTranslation.getHref();
            System.out.println("Translation Success: " + translationHref);

         }
         else
         {
            System.out.println("Translation failed.");
         }

         if (translationHref != null)
         {
            TranslationsGetTranslationResponse translation = onshapeClient.translations().getTranslation().call(colladaTranslation.getId());

            DocumentsDownloadExternalDataResponse dataDownload = onshapeClient.documents().downloadExternalData()
                                                                              .call(translation.getResultExternalDataIds()[0],
                                                                                    translation.getResultDocumentId());

            if (dataDownload != null)
            {
               return new ImmutablePair<>("nadia_" + colladaTranslation.getId() + ".dae", dataDownload.getData());
            }
         }
      }
      catch (OnshapeException e)
      {
         e.printStackTrace();
      }

      return null;
   }
}
