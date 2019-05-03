package us.ihmc.robotics.robotDescription.cad.onShape.javafx.controllers;

import javafx.application.Platform;
import javafx.beans.binding.Bindings;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.PasswordField;
import javafx.scene.control.TextField;
import javafx.scene.layout.BorderPane;
import javafx.scene.transform.Translate;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.node.JavaFXGraphics3DNode;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotDescription.cad.onShape.OnShapeTools;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.concurrent.*;
import java.util.function.Consumer;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OnShapeJavaFXExampleController
{

   @FXML
   private PasswordField accessKeyField;
   @FXML
   private PasswordField secretKeyField;
   @FXML
   private TextField urlField;
   @FXML
   private Button visualizeButton;

   private final ExecutorService executorService;
   private final View3DFactory view3DFactory;
   private final BooleanProperty is3DViewDoingWork;

   private volatile boolean futuresManagerStarted = false;

   private final Map<Future<?>, Consumer<Object>> futuresToManage = new HashMap<>();

   public OnShapeJavaFXExampleController()
   {
      view3DFactory = View3DFactory.createSubscene();

      initializeView3DFactory();

      is3DViewDoingWork = new SimpleBooleanProperty(false);
      executorService = Executors.newFixedThreadPool(4, ThreadTools.getNamedThreadFactory("javafx-background-workers-pool"));

      Thread futuresManagerThread = new Thread(this::manageFutures, "futures-manager");
      futuresManagerThread.setDaemon(true);
      futuresManagerThread.start();
   }

   private void initializeView3DFactory()
   {
      FocusBasedCameraMouseEventHandler cameraController = view3DFactory.addCameraController(true);
      Translate rootJointOffset = new Translate();
      cameraController.prependTransform(rootJointOffset);
   }

   public void initializeController(BorderPane root)
   {
      root.setCenter(view3DFactory.getSubSceneWrappedInsidePane());

      visualizeButton.disableProperty().bind(Bindings.or(accessKeyField.textProperty().isEmpty(), secretKeyField.textProperty().isEmpty())
                                                     .or(urlField.textProperty().isEmpty().or(is3DViewDoingWork)));

      accessKeyField.disableProperty().bind(is3DViewDoingWork);
      secretKeyField.disableProperty().bind(is3DViewDoingWork);
      urlField.disableProperty().bind(is3DViewDoingWork);
   }

   @FXML
   private void onVisualizeAction(ActionEvent event)
   {
      String accessKey = accessKeyField.getText();
      String secretKey = secretKeyField.getText();
      String onshapeURL = urlField.getText();

      OnShapeTools.setCredentials(accessKey, secretKey);

      is3DViewDoingWork.set(true);

      view3DFactory.getRoot().getChildren().clear();

      initializeView3DFactory();

      Callable<Object> task = () -> OnShapeTools.getGraphics3DNodeForURL(onshapeURL).orElse(null);

      submitTaskAndMonitorFuture(task, o -> {
         if (o != null)
         {
            Platform.runLater(() -> view3DFactory.addNodeToView((Node) o));
         }
         is3DViewDoingWork.set(false);
      });
   }

   public void onExit()
   {
      executorService.shutdownNow();
   }

   private <T> void submitTaskAndMonitorFuture(Callable<T> task, Consumer<Object> completionCallback)
   {
      synchronized (futuresToManage)
      {
         Future<?> submit = executorService.submit(task);
         futuresToManage.put(submit, completionCallback);
         futuresToManage.notify();
      }
   }

   private void manageFutures()
   {
      if (!futuresManagerStarted)
      {
         futuresManagerStarted = true;
         while (true)
         {
            try
            {
               synchronized (futuresToManage)
               {
                  if (futuresToManage.isEmpty())
                  {
                     futuresToManage.wait();
                  }

                  HashSet<Future<?>> completedFutures = new HashSet<>();
                  futuresToManage.forEach((future, callback) -> {
                     if (future.isDone())
                     {
                        completedFutures.add(future);

                        Object ret = null;
                        try
                        {
                           ret = future.get();
                        }
                        catch (InterruptedException | ExecutionException e)
                        {
                           e.printStackTrace();
                        }
                        catch (Throwable t)
                        {
                           System.out.println("Some other exception type");
                        }

                        callback.accept(ret);
                     }
                  });

                  completedFutures.forEach(futuresToManage::remove);
               }
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
      }
      else
      {
         LogTools.warn("Futures manager already started, ignoring subsequent calls to #manageFutures()");
      }
   }
}
