plugins {
   id("us.ihmc.ihmc-build") version "0.23.0"
   id("us.ihmc.ihmc-ci") version "7.3"
   id("us.ihmc.ihmc-cd") version "1.17"
}

ihmc {
   group = "us.ihmc"
   version = "0.20.1"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-robot-description"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

mainDependencies {
   api("us.ihmc:euclid:0.15.1")
   api("us.ihmc:ihmc-graphics-description:0.19.1")
}

testDependencies {
   api("us.ihmc:ihmc-commons-testing:0.30.4")
}
