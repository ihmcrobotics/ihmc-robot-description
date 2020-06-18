plugins {
   id("us.ihmc.ihmc-build") version "0.20.1"
   id("us.ihmc.ihmc-ci") version "5.3"
   id("us.ihmc.ihmc-cd") version "1.8"
}

ihmc {
   group = "us.ihmc"
   version = "0.18.0"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-robot-description"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.15.0")
   api("us.ihmc:ihmc-graphics-description:0.17.0-ejml-0.39-beta-1")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.0")
}
