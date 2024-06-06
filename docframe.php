
<meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <meta name="description" content="">
    <meta name="author" content="">
    <link href="images/logos.jpg" rel="shortcut icon">
    <title>PAWFECT Petshop Online Website</title>
	
	<!-- core CSS -->
    <link href="css/bootstrap.css" rel="stylesheet">
    <link href="css/bootstrap.min.css" rel="stylesheet">
    <link href="css/font-awesome.min.css" rel="stylesheet">
    <link href="css/animate.min.css" rel="stylesheet">
    <link href="css/prettyPhoto.css" rel="stylesheet">  
    <link href="css/main.css" rel="stylesheet">
    <link href="css/responsive.css" rel="stylesheet">


<!--*********************************************START OF NAVIGATION BAR****************************************--> 
      
			
            	<table class="table table-responsive table-hover" style="border: 1px dashed #8c8b8b;
                border-top: 1px dashed #8c8b8b;">


         <?php include('includes/dbconn.php');
        $count = 0;
        $did = 0;
        $sql = "SELECT * FROM tbldoc order by did desc" or die (mysqli_error($con));
        
        $result=mysqli_query($con, $sql) or die (mysqli_error($con));
        
        if(mysqli_num_rows($result)>=0){
          while($row = mysqli_fetch_assoc($result)){
            $did = $row['did'];
            $count++;
        ?>




                	<tr style="border: 1px dashed #8c8b8b; cursor:pointer;">
                    <td  style="border: 1px dashed #8c8b8b;"><center><strong class="wow fadeInDown"><p style="margin-top:25px;">No.<?php echo $count;?></p></strong></center></td>
                    	<td style="border: 1px dashed #8c8b8b;"><center><img src="<?php echo $row['image']?>" width="120px;" class="img-responsive img-rounded wow fadeInDown"></center></td>
                        <td style="border: 1px dashed #8c8b8b;"> 
                        <dl class="dl-horizontal wow fadeInDown" style="text-align:left">
                        <dt>Name:</dt> <dd><?php echo $row['name'];?></dd>
						            <dt>Description:</dt> <dd><?php echo $row['description'];?></dd>
                        <dt>Location:</dt> <dd><?php echo $row['location'];?></dd>
                        </dl></td>
                        <td style="border: 1px dashed #8c8b8b;"><button class="btn btn-success  wow fadeInDown" name="order" type="button" style="margin-top:25px;" data-toggle="modal" data-target="#orderModal<?php echo $did;?>"><i class="glyphicon glyphicon-shopping-cart"></i> Book Appointment</button></td>
                    </tr>
                     <!-- Modal -->
<div class="modal fade" id="orderModal<?php echo $did;?>" tabindex="-1" role="dialog" aria-labelledby="myModalLabel">
  <div class="modal-dialog" role="document">
    <div class="modal-content">
      <div class="modal-header">
        <button type="button" class="close" data-dismiss="modal" aria-label="Close"><span aria-hidden="true">&times;</span></button>
        <h4 class="modal-title" id="myModalLabel"><i class="glyphicon glyphicon-edit"></i> YOUR PET INFORMATION</h4>
      </div>
      <form class="form-horizontal" enctype="multipart/form-data" method="post" action="">
      <div class="modal-body">
      <p>Fields with (*) are required</p>
      <div class="row">
       <div class="col-lg-2">
    <label>Customer Name: *</label>
    </div>
    <div class="col-lg-10">
      <input type="text" class="form-control" id="name" name="name" placeholder="Your name" required>
       <input type="hidden" class="form-control" id="did" name="did"  value="<?php echo $did;?>" required>
    </div>
  </div>
  <div class="row">
  <div class="col-lg-2">
    <label>Pet Name: *</label></div>
    <div class="col-lg-10">
      <textarea class="form-control" name="pname" class="form-control" required></textarea>
    </div>
  </div>
  <div class="row">
    <div class="col-lg-2">
    <label>Pet Category (dog/cat/bird): *</label></div>
    <div class="col-lg-10">
      <input type="text" name="pcategory" class="form-control" required placeholder="">
    </div>
  </div>
  <div class="row">
    <div class="col-lg-2">
    <label>Address: *</label></div>
    <div class="col-lg-10">
      <input type="text" name="address" class="form-control" required placeholder="Your address">
    </div>
  <div class="row">
    <div class="col-lg-2">
    <label>Contact: *</label></div>
    <div class="col-lg-10">
      <input type="text" name="contact" class="form-control" required placeholder="Your number">
    </div>
  </div>
    <div class="row">
    <div class="col-lg-2">
    <label>Reason for appointment: *</label></div>
    <div class="col-lg-10">
      <input type="text" name="reason" class="form-control" required placeholder="">
    </div>
  </div>
   <div class="row" id="datepickup">
    <div class="col-lg-2">
    <label>Date of Appointment: *</label></div>
    <div class="col-lg-10">
      <input type="date" name="datep" class="form-control" required/>
    </div>
  </div>
  
 </div>
      <div class="modal-footer">
        
        <button type="submit" class="btn btn-primary" name="savechanges"></i> Book</button>
      </div>
      </form>
    </div>
  </div>
</div>
 <?php }}

 else {echo '<strong style="color:red">No availables data in server</strong>'; } 

 ?>
</table>
            
         
 
<?php include('includes/dbconn.php');
if(isset($_POST['savechanges']))
{
		$aid = $_POST['did'];
		$name = $_POST['name'];
        $pname=$_POST['pname'];
        $pcategory=$_POST['pcategory'];
		$address = $_POST['address'];
		$contact = $_POST['contact'];
		$datep = $_POST['datep'];


	

	$sql = ("INSERT INTO tblapp VALUES($aid,NULL,'$name','$pname','$pcategory','$address','$contact','$datep')") or die (mysqli_error());
    $result=mysqli_query($con, $sql);
							if($result==true){
								echo '<script>alert("Your appointment is booked.The system will follow up by contacting your number thankyou!");
											 window.location.href="docframe.php"</script>';}
											 else{
												 echo '<script>alert("Sorry unable to process your request. please try again later!");
											 window.location.href="docframe.php"</script>';
												 }
	}	
?>
    <script src="js/jquery.js"></script>
    <script src="js/bootstrap.min.js"></script>
    <script src="js/jquery.prettyPhoto.js"></script>
    <script src="js/jquery.isotope.min.js"></script>
    <script src="js/main.js"></script>
    <script src="js/wow.min.js"></script>

 <script type="text/javascript">
    $("#datepickup").hide();
    

   $(document).ready(function(){
    $("select").change(function(){
        $(this).find("option:selected").each(function(){
            if($(this).attr("value")=="Pick-up"){
               
                $("#datepickup").show(200);
               
            }
            else{
                $("#datepickup").hide();
               
            }

        });
    }).change();
    });
  </script>