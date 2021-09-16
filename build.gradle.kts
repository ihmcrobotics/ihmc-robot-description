plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.20"
}

ihmc {
   group = "us.ihmc"
   version = "0.21.2"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-robot-description"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.17.0")
   api("us.ihmc:ihmc-graphics-description:0.19.4")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.4")
}
