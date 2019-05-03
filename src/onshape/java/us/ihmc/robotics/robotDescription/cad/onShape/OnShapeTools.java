package us.ihmc.robotics.robotDescription.cad.onShape;

import com.onshape.api.Onshape;
import com.onshape.api.exceptions.OnshapeException;
import com.onshape.api.requests.AssembliesGetAssemblyDefinitionRequest;
import com.onshape.api.responses.*;
import com.onshape.api.types.Blob;
import com.onshape.api.types.OnshapeDocument;
import com.onshape.api.types.WV;
import javafx.util.Pair;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.log.LogTools;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.*;
import java.util.stream.Collectors;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OnShapeTools
{
   private static final Path modelCacheHome = Paths.get(System.getProperty("user.home"), ".ihmc", "model-cache");
   private static final Onshape apiClient = new Onshape();

   private static String accessKey;
   private static String secretKey;
   private static boolean credentialsSet = false;

   private OnShapeTools()
   {
   }

   public static void setCredentials(String accessKey, String secretKey)
   {
      OnShapeTools.accessKey = accessKey;
      OnShapeTools.secretKey = secretKey;
      apiClient.setAPICredentials(accessKey, secretKey);
      OnShapeTools.credentialsSet = true;
   }

   public static void resetCredentials()
   {
      OnShapeTools.accessKey = null;
      OnShapeTools.secretKey = null;
      apiClient.setAPICredentials(null, null);
      OnShapeTools.credentialsSet = false;
   }

   public static boolean areCredentialsSet()
   {
      return OnShapeTools.credentialsSet;
   }

   public static Optional<JavaFXGraphics3DNode> getGraphics3DNodeForURL(String onshapeURL) throws OnshapeException
   {
      if (!areCredentialsSet())
         return Optional.empty();

      OnshapeDocument onshapeDocument = new OnshapeDocument(onshapeURL);
      AssembliesGetAssemblyDefinitionRequest.Builder assemblyDefinitionBuilder = apiClient.assemblies().getAssemblyDefinition().includeMateConnectors(true)
                                                                                          .includeMateFeatures(true);

      DocumentsGetDocumentResponse documentResponse = apiClient.documents().getDocument().call(onshapeDocument);
      String documentName = documentResponse.getName();

      AssembliesGetAssemblyDefinitionResponse assemblyDefinitionResponse = assemblyDefinitionBuilder.call(onshapeDocument);

      AssembliesGetAssemblyDefinitionResponseRootAssembly rootAssembly = assemblyDefinitionResponse.getRootAssembly();

      Set<String> uniqueDocumentIDs = new HashSet<>();
      List<AssembliesGetAssemblyDefinitionResponseRootAssemblyInstances> uniqueInstances = new ArrayList<>();

      for (AssembliesGetAssemblyDefinitionResponseRootAssemblyInstances instance : rootAssembly.instances)
      {
         if (!uniqueDocumentIDs.contains(instance.documentId))
         {
            uniqueDocumentIDs.add(instance.documentId);
            uniqueInstances.add(instance);
         }
      }

      Path modelCachePath = modelCacheHome.resolve(documentName);
      List<Pair<Path, AssembliesCreateTranslationResponse>> colladaTranslations = uniqueInstances.parallelStream().filter(Objects::nonNull)
                                                                                                 .map(instance -> createColladaTranslationForUncachedAssemblies(
                                                                                                       modelCachePath, instance)).filter(Objects::nonNull)
                                                                                                 .collect(Collectors.toList());

      colladaTranslations.parallelStream().filter(Objects::nonNull).map(OnShapeTools::pollTranslationForDownloadData).filter(Objects::nonNull)
                         .map(OnShapeTools::saveBlobToColladaFile).filter(Objects::nonNull);
      //      Graphics3DObject graphics3dObject = new Graphics3DObject();
      //      graphics3dObject.addModelFile("meshes/pelvis_test.obj");
      //      JavaFXGraphics3DNode vizNode = new JavaFXGraphics3DNode(new Graphics3DNode("vis-node", graphics3dObject));
      //      return Optional.ofNullable(vizNode);

      return Optional.empty();
   }

   private static Path saveBlobToColladaFile(Pair<Path, Blob> pair)
   {
      Path savePath = pair.getKey();
      Blob data = pair.getValue();

      try (FileOutputStream stream = new FileOutputStream(savePath.toFile(), false))
      {
         stream.write(data.getData());
         return savePath;
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      return null;
   }

   private static Pair<Path, Blob> pollTranslationForDownloadData(Pair<Path, AssembliesCreateTranslationResponse> pair)
   {
      Path downloadLocation = pair.getKey();
      AssembliesCreateTranslationResponse translationResponse = pair.getValue();

      boolean translationFailed = false;
      boolean translationDoneOrError = false;
      TranslationsGetTranslationResponse updatedTranslationResponse = null;

      try
      {
         while (!translationDoneOrError)
         {
            updatedTranslationResponse = apiClient.translations().getTranslation().call(translationResponse.getId());
            String requestState = updatedTranslationResponse.getRequestState();

            switch (requestState.toLowerCase())
            {
            case "done":
               translationDoneOrError = true;
               break;
            case "failed":
               translationDoneOrError = true;
               translationFailed = true;
               break;
            default:
               LogTools.info("Request state " + requestState);
               ThreadTools.sleep(1000);
               break;
            }
         }
      }
      catch (OnshapeException e)
      {
         e.printStackTrace();
         return null;
      }

      if (!translationFailed)
      {
         try
         {
            LogTools.info("Translation successful: " + updatedTranslationResponse.getHref());

            DocumentsDownloadExternalDataResponse dataDownload = apiClient.documents().downloadExternalData()
                                                                          .call(updatedTranslationResponse.getResultExternalDataIds()[0],
                                                                                updatedTranslationResponse.getResultDocumentId());
            return new Pair<>(downloadLocation, dataDownload.getData());
         }
         catch (OnshapeException e)
         {
            e.printStackTrace();
            return null;
         }
      }
      else
      {
         LogTools.error("Translation failed: " + updatedTranslationResponse.getFailureReason());
         return null;
      }
   }

   private static Pair<Path, AssembliesCreateTranslationResponse> createColladaTranslationForUncachedAssemblies(Path modelCachePath,
                                                                                                                AssembliesGetAssemblyDefinitionResponseRootAssemblyInstances instance)
   {
      try
      {
         Path meshDownloadDirectoryPath = Paths
               .get(modelCachePath.toString(), instance.documentId, instance.documentVersion, instance.documentMicroversion, instance.elementId);
         Files.createDirectories(meshDownloadDirectoryPath);

         Path meshDownloadLocation = meshDownloadDirectoryPath.resolve(instance.name + ".dae");

         if (Files.exists(meshDownloadLocation))
         {
            LogTools.info("File already cached: " + meshDownloadLocation);
         }
         else
         {
            LogTools.info("Requesting collada mesh for: " + instance.name);
            return new Pair<>(meshDownloadLocation, apiClient.assemblies().createTranslation().formatName("COLLADA")
                                                             .call(instance.documentId, WV.Version, instance.documentVersion, instance.elementId));
         }

      }
      catch (IOException | OnshapeException e)
      {
         e.printStackTrace();
      }

      return null;
   }
}
