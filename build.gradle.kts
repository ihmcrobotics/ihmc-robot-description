plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.23"
}

ihmc {
   group = "us.ihmc"
   version = "0.21.6"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-robot-description"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.19.1")
   api("us.ihmc:ihmc-graphics-description:0.20.0")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.32.0")
}
