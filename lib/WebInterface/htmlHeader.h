R"html(
  <head>
    <title>Encoder board configuration</title>
    <style>

      body {
        font-family: Arial, sans-serif;
        background-color: #111;
        color: #fff;
        margin: 0;
        padding: 0;
        display: flex;
        justify-content: center;
        min-height: 100vh;
      }

      .container {
        display: grid;
        grid-template-columns: repeat(3, 1fr); /* Creates 3 equal-width columns */
        gap: 20px; /* Adjusts spacing between grid items */
        padding: 20px;
      }

      .form-container {
        background-color: #222;
        border-radius: 8px;
        box-shadow: 0px 2px 8px rgba(0, 0, 0, 0.1);
        padding: 20px;
        width: 100%; /* Ensures the form container fills its grid cell */
        box-sizing: border-box;
      }

      .form-group {
        display: flex;
        align-items: center;
      }

      .form-group label {
        margin-right: 10px; /* Adjust the spacing between label and input */
      }

      label {
        font-weight: bold;
        margin-bottom: 8px;
      }

      input[type="text"],
      input[type="submit"],
      input[type="password"],
      select {
        padding: 8px;
        margin-bottom: 12px;
        border: 1px solid #444;
        border-radius: 4px;
        font-size: 14px;
        width: 100%;
        background-color: #333;
        color: #fff;
      }

      .ip-input-container {
        display: flex;
        align-items: center;
      }

      .ip-input-container label {
        margin-right: 4px;
      }

      input[type="submit"] {
        background-color: #007bff;
        border: none;
        cursor: pointer;
        transition: background-color 0.2s ease-in-out;
      }

      input[type="submit"]:hover {
        background-color: #0056b3;
      }

      iframe {
        display: none;
      }

    </style>
  </head>
  )html"