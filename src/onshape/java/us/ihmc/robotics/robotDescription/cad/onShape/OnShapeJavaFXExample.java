package us.ihmc.robotics.robotDescription.cad.onShape;

import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;
import us.ihmc.robotics.robotDescription.cad.onShape.javafx.controllers.OnShapeJavaFXExampleController;

import java.net.URL;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class OnShapeJavaFXExample extends Application
{
   private BorderPane root;
   private OnShapeJavaFXExampleController controller;

   /**
    * The main entry point for all JavaFX applications.
    * The start method is called after the init method has returned,
    * and after the system is ready for the application to begin running.
    *
    * <p>
    * NOTE: This method is called on the JavaFX Application Thread.
    * </p>
    *
    * @param primaryStage the primary stage for this application, onto which
    * the application scene can be set. The primary stage will be embedded in
    * the browser if the application was launched as an applet.
    * Applications may create other stages, if needed, but they will not be
    * primary stages and will not be embedded in the browser.
    */
   @Override
   public void start(Stage primaryStage)
   {
      primaryStage.setTitle("OnShape JavaFX Example");

      Scene scene = new Scene(root, 1280, 720);
      primaryStage.setScene(scene);

      primaryStage.show();
   }

   @FXML
   @Override
   public void init() throws Exception
   {
      super.init();

      String fxmlLocationString = "us/ihmc/robotics/robotDescription/cad/onShape/fxml/" + getClass().getSimpleName() + ".fxml";
      URL fxmlResource = getClass().getClassLoader().getResource(fxmlLocationString);

      if (fxmlResource == null)
      {
         throw new RuntimeException("Could not load FXML " + fxmlLocationString + "!");
      }

      FXMLLoader fxmlLoader = new FXMLLoader(fxmlResource);
      root = fxmlLoader.load();
      controller = fxmlLoader.getController();

      controller.initializeController(root);
   }

   @Override
   public void stop() throws Exception
   {
      controller.onExit();
      super.stop();
   }
}
